"""
Mesin misi otomatis mode 2: dive vertikal otomatis ROV MAIVS EVO.

Modul ini SENGAJA ditulis sebagai Python murni: tidak ada import rclpy,
tidak ada I/O (socket/serial), dan tidak ada pemanggilan waktu sistem
(time.time()/time.monotonic()) di dalamnya. Setiap fungsi menerima waktu
`now` sebagai argumen dari pemanggil. Tujuannya supaya seluruh logika
state-machine dan kendali PID bisa diuji offline dengan data buatan
(lihat test/test_auto_mission.py), tanpa ROV, tanpa ROS, dan tanpa harus
menunggu waktu asli berjalan.

Catatan penting soal arsitektur mode 2 (baca juga CLAUDE.md): di firmware
Teensy (main.cpp), mode 2 adalah ALIAS MURNI dari mode 1 — mixing() thruster
tidak pernah membaca field mode sama sekali, dan applyOutput() hanya
memfilter mode 3 (emergency). Artinya SELURUH logika dive otomatis ada di
sisi GCS (kelas ini), dan firmware tidak diubah sedikit pun: keluaran satu-
satunya dari kelas ini adalah nilai PWM heave (1000-2000) yang nantinya
dikirim lewat field heave pada CSV 10-field yang sama persis dengan yang
dipakai kontrol manual.

Karena ENABLE_IMU_STABILIZATION di firmware bernilai false (lihat main.cpp),
tidak ada stabilisasi pitch/roll aktif sama sekali saat ini. Perubahan PWM
heave yang mendadak bisa membuat ROV miring parah tanpa ada yang
mengoreksinya. Karena itu SLEW LIMIT (AUTO_HEAVE_MAX_STEP) pada heave_pwm
adalah satu-satunya pertahanan anti-tilt yang kita punya, dan ia dipaksakan
di satu tempat saja (fungsi `_slew_toward`, dipanggil dari `tick()`) supaya
tidak ada jalur kode yang bisa melewatkannya secara tidak sengaja.
"""

from dataclasses import dataclass
import statistics
from typing import Optional

# ════════════════════════════════════════════════════════════════════════
# BLOK TUNING — semua konstanta di bawah ini boleh diubah operator sebelum
# lomba/uji coba tanpa perlu memahami isi kelas AutoMission.
# ════════════════════════════════════════════════════════════════════════

# --- Batas target kedalaman (meter) ---
AUTO_TARGET_MIN = 0.10      # target paling dangkal yang boleh diminta
AUTO_TARGET_MAX = 0.70      # target paling dalam yang boleh diminta
AUTO_DEPTH_MARGIN = 0.15    # margin over-depth di atas target_locked

# --- Ambang batas "sudah di permukaan" (meter, relatif terhadap zero) ---
AUTO_SURFACE_DEPTH = 0.05

# --- Kecepatan rayapan setpoint (m/detik) dan waktu tinggal di target ---
AUTO_V_DESC = 0.04
AUTO_V_ASC = 0.05
AUTO_DWELL = 5.0            # detik menahan di target_locked (fase HOLD)

# --- Gain PID kedalaman (dt dalam detik, error dalam meter) ---
AUTO_KP = 700.0
AUTO_KI = 80.0
# AUTO_KD default nol: turunan sensitif terhadap noise +/-3 cm khas
# MS5837-30BA di kolam dangkal.
AUTO_KD = 0.0

# --- Feedforward per fase (offset PWM konstan, sebelum clamp otoritas) ---
AUTO_FF_DESC = 0.0
AUTO_FF_ASC = 0.0

# --- Batas otoritas & anti-tilt ---
# AUTO_HEAVE_AUTHORITY: offset PWM maksimum dari 1500 (heave PID); default
# dipakai arm() bila heave_authority tidak diberikan (lihat arm()).
AUTO_HEAVE_AUTHORITY = 150
AUTO_HEAVE_MAX_STEP = 6      # slew: perubahan heave_pwm maksimum per tick

# Rentang aman untuk heave_authority yang boleh diberikan ke arm() (mis.
# lewat ros2 param auto_heave_authority di mavis_gamepad_node.py). Di luar
# ini DITOLAK, bukan di-clamp diam-diam: batas bawah mencegah operator
# tidak sengaja set ke nilai sangat kecil/0 (heave nyaris/tidak bisa
# bergerak sama sekali), batas atas mencegah melebihi AUTO_HEAVE_AUTHORITY
# default yang sudah divalidasi aman terhadap hardware.
AUTO_HEAVE_AUTHORITY_MIN = 30.0
AUTO_HEAVE_AUTHORITY_MAX = 150.0

# Tanda offset heave: +1 berarti offset POSITIF (heave_pwm > 1500) dipakai
# untuk MENURUNKAN ROV (turun/descend).
#
# ALASAN nilai default ini (WAJIB diverifikasi ulang di air sebelum
# dipercaya — arah dorong fisik tergantung pemasangan baling-baling &
# flag `reversed` tiap ESC di main.cpp, yang tidak bisa dipastikan hanya
# dari membaca kode):
#
# Di mavis_gamepad_node.py baris 218-221, trigger yang diberi nama
# `trig_down` (axis 5) menghasilkan heave_axis POSITIF, yang lewat
# map_pwm() menjadi h_target > 1500 TANPA inversi tanda. Nama variabel
# itu sendiri (dipilih operator yang membuat pemetaan gamepad) adalah
# satu-satunya petunjuk niat arah yang tersedia dari kode: trigger yang
# dimaksudkan untuk "turun" menghasilkan PWM heave di atas netral. Maka
# offset heave positif (heave_pwm > 1500) diasumsikan berarti "turun",
# dan AUTO_HEAVE_SIGN = +1 menjaga hasil PID (yang sudah didefinisikan
# positif = butuh turun lebih dalam, lihat _run_active) tetap konsisten
# dengan konvensi tersebut. Balik ke -1 di sini SEGERA jika uji hardware
# pertama menunjukkan ROV justru naik saat heave_pwm > 1500.
AUTO_HEAVE_SIGN = 1

# --- Filter kedalaman (median-of-5 lalu EMA) & zeroing permukaan ---
AUTO_DEPTH_ALPHA = 0.15
AUTO_ZERO_DURATION = 2.0     # detik mengumpulkan sampel saat ZEROING
AUTO_ZERO_MAX_SPREAD = 0.06  # spread maksimum sampel zeroing (meter)

# --- Ambang batas keamanan / abort ---
AUTO_STALE_TIMEOUT = 0.5     # detik; lewat ini, telemetri depth dianggap mati
AUTO_TILT_LIMIT = 25.0       # derajat, batas |pitch| dan |roll|
AUTO_TILT_GRACE = 0.5        # detik, toleransi spike sebelum benar-benar abort
# Slack tetap (detik) yang ditambahkan ke atas estimasi waktu tempuh
# rayapan setpoint untuk mendapatkan deadline fase DESCEND/ASCEND dan
# anggaran total misi. Deadline-nya SENDIRI dihitung per misi saat arm()
# (lihat arm() dan _enter_state), bukan konstanta tetap — target dalam
# (mis. 0.65 m) butuh rayapan & konvergensi lebih lama daripada target
# dangkal (mis. 0.3 m) karena jarak tempuh setpoint lebih panjang DAN
# buoyancy melawan lebih lama, jadi anggaran waktu yang tetap (dulu
# AUTO_PHASE_TIMEOUT=30s untuk semua target) membuat target dalam
# kehabisan slack jauh lebih cepat daripada target dangkal walau ROV-nya
# sama sekali tidak bermasalah.
AUTO_PHASE_SLACK = 20.0
AUTO_ABORT_AXIS_THRESHOLD = 0.15  # |heave_axis_raw| di atas ini = ambil alih

# Jeda "tahan tombol" sebelum arm() sebaiknya benar-benar dieksekusi oleh
# operator (mis. gamepad harus menahan tombol arm N detik). AutoMission
# sendiri TIDAK memakai konstanta ini — arm() di kelas ini bersifat sekali
# panggil dan instan. Konstanta ini disediakan untuk tahap berikutnya di
# sisi caller (mis. mavis_gamepad_node.py) yang akan mengimplementasikan
# gestur "tahan tombol" sebelum benar-benar memanggil arm().
AUTO_ARM_HOLD_TIME = 1.0

# Lama depth harus stabil di bawah AUTO_SURFACE_DEPTH sebelum SURFACED
# dianggap selesai (-> DONE). Ini bagian dari definisi state machine itu
# sendiri (bukan parameter tuning bebas seperti daftar AUTO_* di atas),
# jadi sengaja tidak diberi awalan AUTO_.
_SURFACE_STABLE_SECONDS = 1.5

# Toleransi "sudah cukup dekat target" sebelum DESCEND boleh pindah ke HOLD.
# Setpoint sendiri merayap berbasis waktu (lihat _update_setpoint), tapi
# fase DESCEND TIDAK boleh selesai hanya karena rayapan setpoint-nya
# selesai — ia harus menunggu kedalaman SUNGGUHAN juga sampai, dibatasi
# oleh deadline fase (lihat _phase_timeout_at). Tanpa syarat ini, ROV yang
# terlalu apung (tidak pernah benar-benar turun) akan tetap "lolos" ke
# HOLD/ASCEND/DONE hanya karena jam rayapan setpoint-nya habis, padahal
# ROV tidak pernah bergerak.
_ARRIVAL_TOLERANCE = 0.05

_TERMINAL_STATES = ('DONE', 'ABORTED', 'FAILED')


def _clamp(value, low, high):
    """Kembalikan value yang dibatasi ke rentang [low, high]."""
    return max(low, min(high, value))


def _slew_toward(target, previous, max_step):
    """Gerakkan previous menuju target maksimum max_step per pemanggilan."""
    delta = target - previous
    if delta > max_step:
        return previous + max_step
    if delta < -max_step:
        return previous - max_step
    return target


@dataclass(frozen=True)
class AutoOutput:
    """Hasil satu langkah tick() AutoMission, siap dikirim sebagai heave."""

    heave_pwm: int
    state: str
    target_locked: Optional[float]
    setpoint_rel: float
    depth_rel: float
    depth_zero: Optional[float]
    elapsed: float
    reason: str
    is_active: bool


class AutoMission:
    """
    State machine dive vertikal otomatis untuk mode 2.

    Urutan status normal:
        IDLE -> ZEROING -> DESCEND -> HOLD -> ASCEND -> SURFACED -> DONE
    Status terminal (DONE, ABORTED, FAILED) hanya bisa ditinggalkan lewat
    reset(). Lihat docstring modul untuk konteks kenapa kelas ini bebas I/O.
    """

    def __init__(self):
        """Inisialisasi AutoMission langsung dalam status IDLE."""
        self.reset()

    def reset(self):
        """Kembali ke IDLE bersih. Satu-satunya jalan keluar dari terminal."""
        self.state = 'IDLE'
        self.target_locked = None
        self.max_depth = None
        self.heave_authority = AUTO_HEAVE_AUTHORITY
        self.depth_zero = None
        self.setpoint_rel = 0.0
        self.reason = ''
        self._mission_start = None
        self._mission_timeout_at = None
        self._descend_phase_budget = None
        self._ascend_phase_budget = None
        self._state_start = None
        self._phase_timeout_at = None
        self._last_tick_time = None
        self._zero_samples = []
        self._zero_override = None
        self._raw_history = []
        self._filtered_depth = None
        self._last_measurement = None
        self._integral = 0.0
        self._prev_heave_pwm = 1500
        self._tilt_bad_since = None
        self._surface_ok_since = None
        self._pending_terminal = None
        self._neutral_settling = False
        self._ascend_start_depth = None

    def arm(self, now, target_depth, heave_authority=None):
        """
        Kunci target_depth (dan heave_authority) lalu mulai misi dari IDLE.

        Mengembalikan False (tanpa mengubah apa pun) bila dipanggil saat
        state bukan IDLE, target_depth kosong, atau di luar rentang
        [AUTO_TARGET_MIN, AUTO_TARGET_MAX] — TIDAK PERNAH di-clamp diam-diam.

        heave_authority opsional: None berarti pakai default modul
        (AUTO_HEAVE_AUTHORITY). Bila diberikan, harus berada di rentang
        [AUTO_HEAVE_AUTHORITY_MIN, AUTO_HEAVE_AUTHORITY_MAX] atau arm()
        ditolak (juga TIDAK di-clamp diam-diam, sama seperti target_depth).
        Nilai yang berhasil dikunci berlaku SEPANJANG misi ini saja —
        perubahan parameter setelah arm() tidak mengubah misi yang sedang
        berjalan (dikunci di self.heave_authority saat arm(), bukan dibaca
        ulang tiap tick).
        """
        if self.state != 'IDLE':
            return False
        if target_depth is None:
            self.reason = 'Target kedalaman kosong, misi ditolak.'
            return False
        if not (AUTO_TARGET_MIN <= target_depth <= AUTO_TARGET_MAX):
            self.reason = (
                'Target {:.2f} m di luar rentang aman {:.2f}-{:.2f} m.'
            ).format(target_depth, AUTO_TARGET_MIN, AUTO_TARGET_MAX)
            return False

        if heave_authority is None:
            heave_authority = AUTO_HEAVE_AUTHORITY
        elif not (AUTO_HEAVE_AUTHORITY_MIN <= heave_authority
                  <= AUTO_HEAVE_AUTHORITY_MAX):
            self.reason = (
                'Otoritas heave {:.1f} di luar rentang aman {:.1f}-{:.1f}.'
            ).format(
                heave_authority, AUTO_HEAVE_AUTHORITY_MIN,
                AUTO_HEAVE_AUTHORITY_MAX)
            return False

        self.target_locked = float(target_depth)
        self.max_depth = self.target_locked + AUTO_DEPTH_MARGIN
        self.heave_authority = float(heave_authority)

        # Anggaran waktu per fase dihitung dari target SAAT INI, bukan
        # konstanta tetap: target dalam butuh rayapan setpoint lebih
        # panjang dan konvergensi lebih lama (buoyancy melawan lebih
        # lama), jadi harus dapat slack yang sebanding, bukan jatah yang
        # sama dengan target dangkal.
        self._descend_phase_budget = (
            self.target_locked / AUTO_V_DESC) + AUTO_PHASE_SLACK
        self._ascend_phase_budget = (
            (self.target_locked - AUTO_SURFACE_DEPTH) / AUTO_V_ASC
        ) + AUTO_PHASE_SLACK
        mission_budget = (
            self._descend_phase_budget + AUTO_DWELL
            + self._ascend_phase_budget + AUTO_PHASE_SLACK)

        self._mission_start = now
        self._mission_timeout_at = now + mission_budget
        self.reason = ''

        if self._zero_override is not None:
            self.depth_zero = self._zero_override
            self._enter_state('DESCEND', now)
        else:
            self._enter_state('ZEROING', now)
        return True

    def set_surface_zero(self, samples):
        """
        Tetapkan titik nol kedalaman manual, melewati fase ZEROING.

        Hanya berlaku dipanggil saat state IDLE. `samples` adalah kumpulan
        pembacaan kedalaman mentah (meter) saat ROV dianggap di permukaan;
        nilainya diambil sebagai median.
        """
        if self.state != 'IDLE':
            return
        samples = list(samples)
        if not samples:
            return
        self._zero_override = statistics.median(samples)

    def tick(self, now, depth_raw, pitch_deg, roll_deg, depth_stamp,
             heave_axis_raw, request_abort):
        """Jalankan satu langkah kendali dan kembalikan AutoOutput."""
        dt = 0.0
        if self._last_tick_time is not None:
            dt = max(0.0, now - self._last_tick_time)
        self._last_tick_time = now

        if self.state == 'IDLE' or self.state in _TERMINAL_STATES:
            return self._make_output(now, 1500)

        # Prioritas tertinggi: tanpa depth yang bisa dipercaya, kita tidak
        # boleh menggerakkan apa pun — bypass slew, langsung netral & abort.
        if now - depth_stamp > AUTO_STALE_TIMEOUT:
            self.reason = (
                'Telemetri kedalaman basi (> {:.2f} s), heave dipaksa '
                'netral seketika tanpa slew demi keamanan.'
            ).format(AUTO_STALE_TIMEOUT)
            self._prev_heave_pwm = 1500
            self._finalize('ABORTED')
            return self._make_output(now, 1500)

        self._update_depth_filter(depth_raw)
        self._check_abort_and_fail(
            now, pitch_deg, roll_deg, heave_axis_raw, request_abort)

        heave_target = 1500
        if self._pending_terminal is None:
            if self.state == 'ZEROING':
                self._run_zeroing(now, depth_raw)
            elif self.state in ('DESCEND', 'HOLD', 'ASCEND'):
                heave_target = self._run_active(now, dt)
            elif self.state == 'SURFACED':
                self._run_surfaced(now)
        elif self.state == 'ASCEND' and not self._neutral_settling:
            heave_target = self._run_active(now, dt)

        if self._neutral_settling:
            heave_target = 1500

        heave_pwm = _slew_toward(
            heave_target, self._prev_heave_pwm, AUTO_HEAVE_MAX_STEP)
        self._prev_heave_pwm = heave_pwm

        if self._neutral_settling and heave_pwm == 1500:
            if self._pending_terminal is not None:
                self._finalize(self._pending_terminal)
            else:
                # Penyelesaian ASCEND jalur normal (bukan abort/gagal):
                # heave sudah netral, lanjut ke SURFACED menunggu
                # konfirmasi stabil, bukan langsung ke status terminal.
                self._neutral_settling = False
                self._enter_state('SURFACED', now)

        return self._make_output(now, heave_pwm)

    # ── Internal: transisi status ────────────────────────────────────────
    def _enter_state(self, new_state, now, is_resume=False):
        self.state = new_state
        self._state_start = now
        self._integral = 0.0
        self._last_measurement = None
        if new_state == 'ZEROING':
            self._zero_samples = []
        elif new_state == 'DESCEND':
            self.setpoint_rel = 0.0
            self._phase_timeout_at = now + self._descend_phase_budget
        elif new_state == 'HOLD':
            self.setpoint_rel = self.target_locked
        elif new_state == 'ASCEND':
            baseline = self._filtered_depth
            if baseline is None:
                baseline = self.setpoint_rel
            self._ascend_start_depth = baseline
            self.setpoint_rel = baseline
            if not is_resume:
                # is_resume=True dipakai saat SURFACED mundur balik ke
                # ASCEND (lihat _run_surfaced) karena ternyata belum betul-
                # betul stabil di permukaan — itu masih upaya ascend yang
                # SAMA, jadi batas waktu fase yang asli tidak boleh diulang
                # dari nol. Tanpa penjagaan ini, ROV yang naik-turun di
                # sekitar AUTO_SURFACE_DEPTH bisa menggantung selamanya
                # karena deadline fase terus di-reset tiap regresi.
                self._phase_timeout_at = now + self._ascend_phase_budget
        elif new_state == 'SURFACED':
            self._surface_ok_since = None

    def _finalize(self, terminal_state):
        self.state = terminal_state
        self._pending_terminal = None
        self._neutral_settling = False

    # ── Internal: filter kedalaman ───────────────────────────────────────
    def _update_depth_filter(self, depth_raw):
        if self.state == 'ZEROING' or self.depth_zero is None:
            return
        self._raw_history.append(depth_raw)
        if len(self._raw_history) > 5:
            self._raw_history.pop(0)
        median_depth = statistics.median(self._raw_history)
        instant = median_depth - self.depth_zero
        if self._filtered_depth is None:
            self._filtered_depth = instant
        else:
            self._filtered_depth += AUTO_DEPTH_ALPHA * (
                instant - self._filtered_depth)

    # ── Internal: deteksi abort & gagal ──────────────────────────────────
    def _check_abort_and_fail(
            self, now, pitch_deg, roll_deg, heave_axis_raw, request_abort):
        if self._pending_terminal is not None:
            return

        fail_reason = None
        if self.state in ('DESCEND', 'ASCEND'):
            if (self._phase_timeout_at is not None
                    and now >= self._phase_timeout_at):
                budget = (self._descend_phase_budget
                          if self.state == 'DESCEND'
                          else self._ascend_phase_budget)
                fail_reason = (
                    'Fase {} melebihi anggaran waktu {:.1f} s '
                    '(target {:.2f} m).'
                ).format(self.state, budget, self.target_locked)
        if (fail_reason is None and self._mission_timeout_at is not None
                and now >= self._mission_timeout_at):
            fail_reason = (
                'Total durasi misi melebihi anggaran waktu (target '
                '{:.2f} m).'
            ).format(self.target_locked)
        if fail_reason is not None:
            self.reason = fail_reason
            self._pending_terminal = 'FAILED'
            self._neutral_settling = True
            return

        abort_reason = None
        if request_abort:
            abort_reason = 'Diminta operator (request_abort).'
        elif abs(heave_axis_raw) > AUTO_ABORT_AXIS_THRESHOLD:
            abort_reason = 'Operator mengambil alih sumbu heave manual.'
        elif (self._filtered_depth is not None and self.max_depth is not None
                and self._filtered_depth > self.max_depth):
            abort_reason = (
                'Kedalaman {:.2f} m melewati batas aman {:.2f} m.'
            ).format(self._filtered_depth, self.max_depth)
        elif self._filtered_depth is not None:
            tilt_bad = (abs(pitch_deg) > AUTO_TILT_LIMIT
                        or abs(roll_deg) > AUTO_TILT_LIMIT)
            if tilt_bad:
                if self._tilt_bad_since is None:
                    self._tilt_bad_since = now
                elif now - self._tilt_bad_since > AUTO_TILT_GRACE:
                    abort_reason = 'Kemiringan ROV melewati batas terlalu lama.'
            else:
                self._tilt_bad_since = None

        if abort_reason is not None:
            self.reason = abort_reason
            self._pending_terminal = 'ABORTED'
            if self.state in ('ZEROING', 'SURFACED'):
                # Di kedua status ini heave memang sudah/seharusnya netral,
                # jadi aman langsung dianggap "sedang menetralkan" supaya
                # tidak menggantung menunggu kondisi yang tidak akan pernah
                # tercapai (lihat _run_active, hanya dipanggil utk ASCEND).
                self._neutral_settling = True
            elif self.state in ('DESCEND', 'HOLD'):
                self._enter_state('ASCEND', now)
            # Kalau sudah ASCEND: biarkan _run_active menuntaskan rayapan
            # ke permukaan sendiri, _neutral_settling akan menyusul otomatis.

    # ── Internal: fase ZEROING ───────────────────────────────────────────
    def _run_zeroing(self, now, depth_raw):
        self._zero_samples.append(depth_raw)
        if now - self._state_start < AUTO_ZERO_DURATION:
            return
        spread = max(self._zero_samples) - min(self._zero_samples)
        if spread > AUTO_ZERO_MAX_SPREAD:
            self.reason = (
                'Sensor kedalaman terlalu berisik saat zeroing '
                '(spread {:.3f} m > {:.3f} m).'
            ).format(spread, AUTO_ZERO_MAX_SPREAD)
            self._pending_terminal = 'FAILED'
            self._neutral_settling = True
            return
        self.depth_zero = statistics.median(self._zero_samples)
        self._enter_state('DESCEND', now)

    # ── Internal: fase DESCEND / HOLD / ASCEND ───────────────────────────
    def _run_active(self, now, dt):
        self._update_setpoint(now)
        if self._neutral_settling or self._filtered_depth is None:
            return 1500

        error = self.setpoint_rel - self._filtered_depth
        derivative = 0.0
        if dt > 0.0 and self._last_measurement is not None:
            # Derivative-on-measurement: turunan dari PENGUKURAN, bukan
            # error, supaya transisi setpoint antar fase (DESCEND->HOLD->
            # ASCEND) tidak memicu lonjakan turunan yang tidak nyata.
            derivative = -(self._filtered_depth - self._last_measurement) / dt
        self._last_measurement = self._filtered_depth

        self._integral += error * dt
        # Anti-windup ala main.cpp computeDepthPID: klem akumulator integral
        # langsung ke batas otoritas (bukan presisi dimensional, tapi
        # konsisten dengan gaya firmware yang sudah ada). Pakai
        # self.heave_authority (dikunci saat arm(), lihat arm()), BUKAN
        # konstanta modul AUTO_HEAVE_AUTHORITY langsung -- supaya otoritas
        # custom per-misi benar-benar diberlakukan di sini.
        self._integral = _clamp(
            self._integral, -self.heave_authority, self.heave_authority)

        feedforward = 0.0
        if self.state == 'DESCEND':
            feedforward = AUTO_FF_DESC
        elif self.state == 'ASCEND':
            feedforward = AUTO_FF_ASC

        output = ((AUTO_KP * error) + (AUTO_KI * self._integral)
                  + (AUTO_KD * derivative) + feedforward)
        output = _clamp(output, -self.heave_authority, self.heave_authority)

        return 1500 + int(round(AUTO_HEAVE_SIGN * output))

    def _update_setpoint(self, now):
        if self.state == 'DESCEND':
            self.setpoint_rel = min(
                self.target_locked, AUTO_V_DESC * (now - self._state_start))
            # Pindah ke HOLD butuh DUA syarat: rayapan setpoint selesai DAN
            # kedalaman sungguhan sudah cukup dekat target (_ARRIVAL_
            # TOLERANCE). Kalau hanya syarat pertama, ROV yang terlalu
            # apung/berat akan "lolos" ke HOLD walau tidak pernah benar-
            # benar sampai — deadline fase (_phase_timeout_at)-lah yang
            # menjadi jaring pengaman FAILED bila syarat kedua tidak
            # pernah terpenuhi.
            close_enough = (
                self._filtered_depth is not None
                and abs(self._filtered_depth - self.target_locked)
                <= _ARRIVAL_TOLERANCE)
            if self.setpoint_rel >= self.target_locked and close_enough:
                self._enter_state('HOLD', now)
        elif self.state == 'HOLD':
            if now - self._state_start >= AUTO_DWELL:
                self._enter_state('ASCEND', now)
        elif self.state == 'ASCEND':
            elapsed = now - self._state_start
            self.setpoint_rel = max(
                AUTO_SURFACE_DEPTH,
                self._ascend_start_depth - AUTO_V_ASC * elapsed)
            if self.setpoint_rel <= AUTO_SURFACE_DEPTH:
                self._neutral_settling = True

    # ── Internal: fase SURFACED ──────────────────────────────────────────
    def _run_surfaced(self, now):
        if (self._filtered_depth is not None
                and self._filtered_depth < AUTO_SURFACE_DEPTH):
            if self._surface_ok_since is None:
                self._surface_ok_since = now
            elif now - self._surface_ok_since >= _SURFACE_STABLE_SECONDS:
                self._pending_terminal = 'DONE'
                self._neutral_settling = True
        else:
            self._surface_ok_since = None
            # Ternyata turun lagi dari permukaan: kembali ke ASCEND aktif
            # daripada tetap netral pasif menunggu kondisi yang mundur.
            # is_resume=True: jangan reset deadline fase ASCEND, ini masih
            # upaya ascend yang sama.
            self._enter_state('ASCEND', now, is_resume=True)

    # ── Internal: bentuk output ──────────────────────────────────────────
    def _make_output(self, now, heave_pwm):
        elapsed = 0.0
        if self._mission_start is not None:
            elapsed = now - self._mission_start
        depth_rel = (self._filtered_depth
                     if self._filtered_depth is not None else 0.0)
        return AutoOutput(
            heave_pwm=heave_pwm,
            state=self.state,
            target_locked=self.target_locked,
            setpoint_rel=self.setpoint_rel,
            depth_rel=depth_rel,
            depth_zero=self.depth_zero,
            elapsed=elapsed,
            reason=self.reason,
            is_active=self.state not in (('IDLE',) + _TERMINAL_STATES),
        )
