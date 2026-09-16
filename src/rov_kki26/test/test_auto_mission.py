"""
Uji offline AutoMission (mode 2) memakai model ROV orde-satu buatan.

Model ROV di sini SENGAJA sederhana (bukan simulator fisik sungguhan):
laju perubahan kedalaman sebanding dengan offset heave dikurangi buoyancy
konstan, dilewatkan lag orde-satu (meniru drag/inersia), lalu pembacaan
sensor diberi noise gaussian ~3 cm meniru karakteristik MS5837-30BA di
kolam dangkal. Tujuannya memverifikasi PERILAKU state machine & anti-tilt
AutoMission, bukan mengklaim akurasi hidrodinamika.
"""

import random

import pytest

from rov_kki26.auto_mission import (
    AUTO_ABORT_AXIS_THRESHOLD,
    AUTO_DEPTH_MARGIN,
    AUTO_HEAVE_AUTHORITY,
    AUTO_HEAVE_MAX_STEP,
    AUTO_STALE_TIMEOUT,
    AUTO_TARGET_MAX,
    AUTO_TARGET_MIN,
    AUTO_TILT_LIMIT,
    AutoMission,
)

_NEUTRAL_STATES = ('IDLE', 'ZEROING', 'SURFACED', 'DONE', 'ABORTED', 'FAILED')


class _SimpleRovPlant:
    """Model orde-satu: laju depth ~ (dorongan - buoyancy), dilewat lag."""

    def __init__(self, k_thrust=0.00083, buoyancy_rate=0.01, tau=0.5,
                 noise_sigma=0.03, seed=1234):
        self.k_thrust = k_thrust
        self.buoyancy_rate = buoyancy_rate
        self.tau = tau
        self.noise_sigma = noise_sigma
        self.rng = random.Random(seed)
        self.depth = 0.0
        self.depth_rate = 0.0

    def step(self, heave_pwm, dt):
        """Majukan simulasi satu langkah dt detik memakai heave_pwm."""
        target_rate = (self.k_thrust * (heave_pwm - 1500)
                       - self.buoyancy_rate)
        blend = min(1.0, dt / self.tau) if self.tau > 0 else 1.0
        self.depth_rate += (target_rate - self.depth_rate) * blend
        self.depth = max(0.0, self.depth + self.depth_rate * dt)
        if self.depth <= 0.0 and self.depth_rate < 0.0:
            self.depth_rate = 0.0

    def measure(self):
        """Kembalikan pembacaan kedalaman + noise gaussian ~3 cm."""
        return self.depth + self.rng.gauss(0.0, self.noise_sigma)


def _run_closed_loop(
        target_depth, plant=None, dt=0.1, max_ticks=3000, pitch=0.0,
        roll=0.0, heave_authority=None):
    """Jalankan AutoMission + plant sampai status terminal atau habis tick."""
    plant = plant if plant is not None else _SimpleRovPlant()
    mission = AutoMission()
    mission.set_surface_zero([0.0, 0.0, 0.0, 0.0, 0.0])
    assert mission.arm(0.0, target_depth, heave_authority=heave_authority)

    now = 0.0
    outputs = []
    depths = []
    for _ in range(max_ticks):
        now += dt
        depth_raw = plant.measure()
        output = mission.tick(now, depth_raw, pitch, roll, now, 0.0, False)
        plant.step(output.heave_pwm, dt)
        outputs.append(output)
        depths.append(plant.depth)
        if output.state in ('DONE', 'ABORTED', 'FAILED'):
            break
    return mission, outputs, depths


def _run_scripted(
        mission, now, dt, depth_fn, pitch_fn=None, roll_fn=None,
        heave_axis_fn=None, abort_fn=None, ticks=None,
        stop_on_terminal=True, depth_stamp_fn=None):
    """Jalankan tick() dengan data hasil fungsi buatan, bukan plant fisik."""
    pitch_fn = pitch_fn or (lambda t: 0.0)
    roll_fn = roll_fn or (lambda t: 0.0)
    heave_axis_fn = heave_axis_fn or (lambda t: 0.0)
    abort_fn = abort_fn or (lambda t: False)
    depth_stamp_fn = depth_stamp_fn or (lambda t: t)

    outputs = []
    t = now
    for _ in range(ticks):
        t += dt
        output = mission.tick(
            t, depth_fn(t), pitch_fn(t), roll_fn(t), depth_stamp_fn(t),
            heave_axis_fn(t), abort_fn(t))
        outputs.append(output)
        if stop_on_terminal and output.state in ('DONE', 'ABORTED', 'FAILED'):
            break
    return outputs


def test_normal_mission_completes_near_target():
    """Misi normal harus berakhir DONE dengan puncak dekat target."""
    target = 0.3
    mission, outputs, depths = _run_closed_loop(target)

    assert mission.state == 'DONE'
    peak_depth = max(depths)
    assert peak_depth == pytest.approx(target, abs=0.05)


def test_normal_mission_completes_near_target_deep():
    """
    Misi ke target dalam (0.65 m) juga harus DONE, bukan FAILED.

    Ini kasus yang tadinya rawan gagal palsu saat AUTO_PHASE_TIMEOUT masih
    konstanta tetap: target dalam butuh rayapan setpoint & konvergensi
    lebih lama, jadi slack tetap 30 s menyusut relatif terhadap target
    dangkal. Sejak deadline fase dihitung dari target_locked saat arm(),
    target dalam mendapat anggaran waktu yang sebanding.

    Toleransi puncak dilebarkan ke 0.08 m (bukan 0.05 m seperti test target
    dangkal) berdasarkan pengukuran nyata: sweep 29 seed pada target 0.65 m
    menghasilkan overshoot terburuk ~0.064 m — sedikit lebih besar daripada
    target dangkal karena integral sempat menumpuk lebih lama selama
    DESCEND yang lebih panjang. ROV tetap konvergen (DONE), bukan gagal.
    """
    target = 0.65
    mission, outputs, depths = _run_closed_loop(target, max_ticks=3000)

    assert mission.state == 'DONE'
    peak_depth = max(depths)
    assert peak_depth == pytest.approx(target, abs=0.08)


def test_heave_never_exceeds_slew_limit():
    """Heave tidak boleh melompat lebih dari AUTO_HEAVE_MAX_STEP per tick."""
    _, outputs, _ = _run_closed_loop(0.3)
    heaves = [o.heave_pwm for o in outputs]
    for prev, cur in zip(heaves, heaves[1:]):
        assert abs(cur - prev) <= AUTO_HEAVE_MAX_STEP


def test_heave_slew_respected_during_abort_and_transitions():
    """Slew tetap dipatuhi walau lewat abort (bukan jalur telemetri basi)."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.5)

    # Fase pertama: DESCEND normal beberapa detik supaya heave sempat jauh
    # dari netral, baru operator mengambil alih sumbu heave (abort).
    outputs = _run_scripted(
        mission, 0.0, 0.1, depth_fn=lambda t: 0.0, ticks=30)
    outputs += _run_scripted(
        mission, outputs[-1].elapsed, 0.1, depth_fn=lambda t: 0.05,
        heave_axis_fn=lambda t: 0.9, ticks=400)

    heaves = [o.heave_pwm for o in outputs]
    for prev, cur in zip(heaves, heaves[1:]):
        assert abs(cur - prev) <= AUTO_HEAVE_MAX_STEP
    assert outputs[-1].state == 'ABORTED'
    assert any(o.state == 'ASCEND' for o in outputs)


def test_heave_slew_respected_through_surfaced_ascend_resume():
    """
    Regresi SURFACED->ASCEND (is_resume) tetap mematuhi slew limit.

    Skenario: ROV sempat SURFACED, lalu "tenggelam lagi" sebelum status
    stabil 1.5 detik terpenuhi, memaksa AutoMission mundur balik ke ASCEND
    lewat _enter_state(..., is_resume=True). Ini jalur yang belum disentuh
    test lain, jadi diverifikasi terpisah di sini.
    """
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    # Target lebih dalam dari test lain di file ini (bukan 0.1 m) supaya
    # anggaran fase ASCEND (kini dihitung dari target, lihat arm()) punya
    # slack yang wajar untuk skenario "sempat surfaced, tenggelam sebentar,
    # naik lagi" di bawah — bukan angka sembarang.
    assert mission.arm(0.0, 0.3)

    dt = 0.05
    now = 0.0
    depth_val = 0.3
    heaves = []
    states = []
    surfaced_ticks = 0
    spiked = False
    spike_ticks_left = 0

    for _ in range(2000):
        now += dt
        state = mission.state
        if state == 'ASCEND':
            depth_val = max(0.02, depth_val - 0.06 * dt)
        elif state == 'SURFACED':
            if not spiked:
                surfaced_ticks += 1
                if surfaced_ticks >= 6:
                    # Sempat stabil sebentar (< 1.5 s), lalu "tenggelam
                    # lagi" (lonjakan KECIL & singkat, sekadar melewati
                    # AUTO_SURFACE_DEPTH) supaya _run_surfaced mundur balik
                    # ke ASCEND — lonjakan besar/lama membuat filter EMA
                    # butuh puluhan detik menetap, bukan yang mau diuji di
                    # sini.
                    spiked = True
                    spike_ticks_left = 6
                    depth_val = 0.07
                else:
                    depth_val = 0.02
            elif spike_ticks_left > 0:
                depth_val = 0.07
                spike_ticks_left -= 1
            else:
                depth_val = 0.02

        output = mission.tick(now, depth_val, 0.0, 0.0, now, 0.0, False)
        heaves.append(output.heave_pwm)
        states.append(output.state)
        if output.state == 'DONE':
            break

    for prev, cur in zip(heaves, heaves[1:]):
        assert abs(cur - prev) <= AUTO_HEAVE_MAX_STEP

    first_surfaced = states.index('SURFACED')
    assert any(s == 'ASCEND' for s in states[first_surfaced + 1:]), (
        'regresi SURFACED->ASCEND tidak pernah terjadi di skenario ini')
    assert states[-1] == 'DONE'


def test_heave_never_exceeds_authority():
    """Offset heave dari netral tidak boleh melebihi AUTO_HEAVE_AUTHORITY."""
    _, outputs, _ = _run_closed_loop(0.3)
    for o in outputs:
        assert abs(o.heave_pwm - 1500) <= AUTO_HEAVE_AUTHORITY


def test_custom_heave_authority_respected_when_smaller_than_default():
    """
    arm() dengan heave_authority < default membatasi offset lebih ketat.

    Slew limit (AUTO_HEAVE_MAX_STEP) HARUS tetap dipatuhi persis walau
    otoritas dipersempit -- keduanya independen (slew membatasi laju
    perubahan per tick, authority membatasi besar offset absolut).
    """
    custom_authority = 100.0
    assert custom_authority < AUTO_HEAVE_AUTHORITY

    mission, outputs, _ = _run_closed_loop(
        0.3, heave_authority=custom_authority)

    assert mission.heave_authority == pytest.approx(custom_authority)
    heaves = [o.heave_pwm for o in outputs]
    for h in heaves:
        # Batas KUSTOM (lebih ketat), bukan AUTO_HEAVE_AUTHORITY default.
        assert abs(h - 1500) <= custom_authority
    for prev, cur in zip(heaves, heaves[1:]):
        assert abs(cur - prev) <= AUTO_HEAVE_MAX_STEP
    assert mission.state == 'DONE'


def test_arm_rejects_heave_authority_out_of_range():
    """heave_authority di luar [MIN, MAX] ditolak, bukan di-clamp diam-diam."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3, heave_authority=10.0) is False
    assert mission.state == 'IDLE'
    assert mission.target_locked is None

    mission2 = AutoMission()
    mission2.set_surface_zero([0.0])
    assert mission2.arm(0.0, 0.3, heave_authority=200.0) is False
    assert mission2.state == 'IDLE'
    assert mission2.target_locked is None


def test_heave_neutral_in_idle_zeroing_surfaced_and_terminal():
    """Heave harus persis 1500 di IDLE/ZEROING/SURFACED/status terminal."""
    idle_mission = AutoMission()
    idle_output = idle_mission.tick(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, False)
    assert idle_output.heave_pwm == 1500

    zeroing_mission = AutoMission()
    assert zeroing_mission.arm(0.0, 0.3)
    assert zeroing_mission.state == 'ZEROING'
    zeroing_outputs = _run_scripted(
        zeroing_mission, 0.0, 0.1, depth_fn=lambda t: 0.0, ticks=15,
        stop_on_terminal=False)
    for o in zeroing_outputs:
        assert o.heave_pwm == 1500

    _, outputs, _ = _run_closed_loop(0.3)
    for o in outputs:
        if o.state in ('SURFACED', 'DONE'):
            assert o.heave_pwm == 1500


def test_arm_without_target_is_rejected():
    """arm() tanpa target_depth harus ditolak dan tidak mengubah state."""
    mission = AutoMission()
    assert mission.arm(0.0, None) is False
    assert mission.state == 'IDLE'
    assert mission.target_locked is None


def test_arm_out_of_range_is_rejected():
    """arm() dengan target di luar rentang aman harus ditolak, bukan clamp."""
    mission = AutoMission()
    assert mission.arm(0.0, AUTO_TARGET_MIN - 0.01) is False
    assert mission.state == 'IDLE'
    assert mission.target_locked is None

    mission2 = AutoMission()
    assert mission2.arm(0.0, AUTO_TARGET_MAX + 0.01) is False
    assert mission2.state == 'IDLE'
    assert mission2.target_locked is None


def test_max_depth_tracks_locked_target():
    """max_depth harus mengikuti target_locked, bukan konstanta tetap."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)
    assert mission.max_depth == pytest.approx(0.3 + AUTO_DEPTH_MARGIN)

    mission.reset()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.6)
    assert mission.max_depth == pytest.approx(0.6 + AUTO_DEPTH_MARGIN)


def test_over_depth_triggers_ascend_then_aborted():
    """Kedalaman melewati max_depth harus lewat ASCEND lalu ABORTED."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)
    max_depth = mission.max_depth

    outputs = _run_scripted(
        mission, 0.0, 0.1,
        depth_fn=lambda t: min(max_depth + 0.3, t * 0.05),
        ticks=600)

    assert any(o.state == 'ASCEND' for o in outputs)
    assert outputs[-1].state == 'ABORTED'
    assert 'batas aman' in outputs[-1].reason


def test_stale_telemetry_forces_immediate_neutral():
    """Telemetri depth basi harus langsung memaksa heave 1500 & ABORTED."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)

    outputs = _run_scripted(
        mission, 0.0, 0.1, depth_fn=lambda t: 0.0, ticks=20,
        stop_on_terminal=False)
    prev_heave = outputs[-1].heave_pwm
    assert prev_heave != 1500  # pastikan sedang tidak netral saat basi

    last_now = 2.0
    stale_now = last_now + AUTO_STALE_TIMEOUT + 0.2
    stale_output = mission.tick(
        stale_now, 0.0, 0.0, 0.0, last_now, 0.0, False)

    assert stale_output.heave_pwm == 1500
    assert stale_output.state == 'ABORTED'
    assert 'basi' in stale_output.reason


def test_tilt_spike_does_not_abort():
    """Lonjakan tilt sesaat (di bawah AUTO_TILT_GRACE) tidak boleh abort."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)

    def pitch_fn(t):
        return AUTO_TILT_LIMIT + 10.0 if 0.5 <= t < 0.55 else 0.0

    outputs = _run_scripted(
        mission, 0.0, 0.05, depth_fn=lambda t: 0.0, pitch_fn=pitch_fn,
        ticks=40, stop_on_terminal=False)
    assert all(o.state not in ('ABORTED', 'FAILED') for o in outputs)


def test_tilt_sustained_beyond_grace_triggers_abort():
    """Tilt yang bertahan melewati AUTO_TILT_GRACE harus memicu abort."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)

    outputs = _run_scripted(
        mission, 0.0, 0.05, depth_fn=lambda t: 0.1,
        pitch_fn=lambda t: AUTO_TILT_LIMIT + 10.0, ticks=200)

    assert outputs[-1].state == 'ABORTED'
    assert 'Kemiringan' in outputs[-1].reason


def test_too_buoyant_rov_fails_instead_of_hanging():
    """ROV yang tak pernah sampai target harus berakhir FAILED, bukan hang."""
    plant = _SimpleRovPlant(k_thrust=0.0005, buoyancy_rate=0.5)
    mission, outputs, _ = _run_closed_loop(
        0.3, plant=plant, dt=0.5, max_ticks=400)

    assert mission.state == 'FAILED'
    assert outputs[-1].state == 'FAILED'
    assert outputs[-1].heave_pwm == 1500


def test_noise_alone_never_triggers_false_abort():
    """Noise sensor 3 cm sendirian tidak boleh pernah memicu abort."""
    plant = _SimpleRovPlant(noise_sigma=0.03)
    mission, outputs, _ = _run_closed_loop(0.3, plant=plant)
    assert mission.state == 'DONE'
    assert all(o.state != 'ABORTED' for o in outputs)


def test_arm_ignored_when_not_idle():
    """arm() dipanggil saat bukan IDLE harus diabaikan (return False)."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)
    assert mission.state == 'DESCEND'

    assert mission.arm(1.0, 0.5) is False
    assert mission.target_locked == pytest.approx(0.3)
    assert mission.state == 'DESCEND'


def test_target_change_mid_mission_does_not_affect_running_mission():
    """arm() ulang di tengah misi tidak boleh mengubah target yang berjalan."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)
    max_depth_before = mission.max_depth
    target_before = mission.target_locked

    ignored = mission.arm(0.5, 0.6)

    assert ignored is False
    assert mission.target_locked == pytest.approx(target_before)
    assert mission.max_depth == pytest.approx(max_depth_before)


def test_abort_axis_threshold():
    """Sentuhan sumbu heave manual di atas ambang batas memicu abort."""
    mission = AutoMission()
    mission.set_surface_zero([0.0])
    assert mission.arm(0.0, 0.3)

    outputs = _run_scripted(
        mission, 0.0, 0.1, depth_fn=lambda t: 0.0,
        heave_axis_fn=lambda t: AUTO_ABORT_AXIS_THRESHOLD + 0.5, ticks=400)

    assert outputs[-1].state == 'ABORTED'
    assert 'heave manual' in outputs[-1].reason
