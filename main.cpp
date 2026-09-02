// ============================================================
//  ROV Main Controller (FINAL OPTIMIZED + ARMING LOGIC)
//  Board  : Teensy 4.1
//  Sensor : MPU6050 (IMU) + HMC5883 (Compass) + MS5837 (Depth)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <MS5837.h>
#include <Servo.h>

// ── PWM & SERVO ──────────────────────────────────────────────
#define PWM_MIN           1100
#define PWM_MAX           1900
#define PWM_NEUTRAL       1500
#define SERIAL_TIMEOUT_MS  500   // reset ke netral jika putus dari Jetson

// ── DEPTH HOLD & IMU STABILIZATION ───────────────────────────
// Nilai gain berikut adalah default konservatif (aman untuk uji awal).
// WAJIB dituning ulang setelah hardware terpasang di robot.
// ============================================================
// DEPTH HOLD PID
// ============================================================
const float DEPTH_KP = 150.0f;
const float DEPTH_KI = 5.0f;
const float DEPTH_KD = 8.0f;

const float ANGLE_KP = 5.0f;
const float ANGLE_KI = 0.1f;
const float ANGLE_KD = 1.0f;

const int   MAX_DEPTH_CORRECTION = 200;  // batas offset PWM dari depth-hold
const int   MAX_ANGLE_CORRECTION = 200;  // batas offset PWM dari stabilisasi pitch/roll
const int   HEAVE_DEADBAND       = 30;   // dipertahankan untuk kompatibilitas

// Deadband kedalaman agar PID tidak terus memburu nilai yang sangat dekat
// dengan setpoint. 0.015 m = ±1.5 cm.
const float DEPTH_DEADBAND = 0.015f;

// Balik menjadi -1.0f bila arah koreksi depth ternyata terbalik
// setelah pengujian hardware.
const float DEPTH_OUTPUT_SIGN = 1.0f;

// Set false untuk menonaktifkan sementara stabilisasi pitch/roll (mis. saat gain
// masih terlalu kasar / belum dituning). Depth-hold TIDAK terpengaruh oleh flag ini.
// Koreksi tetap dihitung & ditampilkan di telemetry untuk referensi tuning,
// hanya tidak disalurkan ke mixing thruster.
const bool ENABLE_IMU_STABILIZATION = false;

const uint8_t PIN_DKIRI    = 11;  //B2
const uint8_t PIN_DKANAN   = 10; //A2
const uint8_t PIN_BKIRI    = 12; //A3
const uint8_t PIN_BKANAN   = 24;  //B3
const uint8_t PIN_TKIRI    = 8; //A1
const uint8_t PIN_TKANAN   = 9;  //B1 

const uint8_t PIN_RELAY    = 23; 
const uint8_t PIN_TILT_ARM = 14; 
const uint8_t PIN_GRIPPER  = 15;

// ── Struct ────────────────────────────────────────────────────
struct AutoCommand {
    int surge   = PWM_NEUTRAL; 
    int yaw     = PWM_NEUTRAL; 
    int roll    = PWM_NEUTRAL; 
    int tilt    = PWM_NEUTRAL; 
    int heave   = PWM_NEUTRAL; 
    int tiltArm = 180;         // FIX 360: Diubah dari 0 ke 180 (Tengah)
    int gripper = 180;         // FIX 360: Diubah dari 90 ke 180 (Tengah)
    int mode    = 1;
    int depthHold = 0;         // 1 = depth-hold aktif (dikirim GCS via toggle B0)
    int relevel   = 0;         // 1 = perintah re-zero setpoint IMU (momentari, B3)
};

struct ThrusterOutput {
    int DKIRI, DKANAN, BKIRI, BKANAN, TKIRI, TKANAN;
};

// ── Thruster Class ────────────────────────────────────────────
class Thruster {
public:
    void attach(uint8_t pin, bool reversed = false) {
        this->reversed = reversed;
        esc.attach(pin, PWM_MIN, PWM_MAX);
        esc.writeMicroseconds(PWM_NEUTRAL);
    }
    void write(int value) {
        value = constrain(value, PWM_MIN, PWM_MAX);
        if (reversed) value = PWM_NEUTRAL * 2 - value;
        esc.writeMicroseconds(value);
    }
private:
    Servo esc;
    bool  reversed = false;
};

// ── Global Objects ────────────────────────────────────────────
Adafruit_MPU6050         imu;
Adafruit_HMC5883_Unified hmc5883(12345);
MS5837                   depthSensor;

Thruster tDKIRI, tDKANAN, tBKIRI, tBKANAN, tTKIRI, tTKANAN;
Servo servoTiltArm;
Servo servoGripper;

AutoCommand   autoCmd;
unsigned long lastAutoSerial = 0;
unsigned long lastTime       = 0;
unsigned long lastTaskTime   = 0;
const unsigned long taskInterval = 20; // 50Hz Control Loop

const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

// Flag Arming untuk mencegah relay tertrigger saat pertama kali dihidupkan
bool systemArmed = false; 

// ── Mahony State ──────────────────────────────────────────────
float twoKp = 1.0f;
float twoKi = 0.0f;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;

// ── Depth Hold State ─────────────────────────────────────────
bool  prevDepthHoldFlag = false;
float depthSetpoint     = 0;
float depthIntegral     = 0;
float lastDepthError    = 0;
float depthError        = 0;
float depthCorrection   = 0;

// ── IMU Stabilization State ──────────────────────────────────
bool  prevRelevelFlag = false;
float pitchSetpoint   = 0;   // di-set saat boot & saat relevel (B3)
float rollSetpoint    = 0;
float pitchIntegral   = 0, rollIntegral = 0;
float lastPitchError  = 0, lastRollError = 0;
bool  attitudeSetpointInit = false; // untuk auto-set setpoint pertama kali saat data IMU pertama valid

// ════════════════════════════════════════════════════════════
//  MAHONY AHRS
// ════════════════════════════════════════════════════════════
void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    float recipNorm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    if ((ax != 0) || (ay != 0) || (az != 0)) {
        recipNorm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = 1.0f / sqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        hx = 2*mx*(0.5f - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
        hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5f - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
        bx = sqrt(hx*hx + hy*hy);
        bz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5f - q1*q1 - q2*q2);

        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
        wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
        wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5f - q1*q1 - q2*q2);

        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;

        gx += integralFBx + twoKp * ex;
        gy += integralFBy + twoKp * ey;
        gz += integralFBz + twoKp * ez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// ════════════════════════════════════════════════════════════
//  PID GENERIK (dengan anti-windup sederhana)
// ════════════════════════════════════════════════════════════
float computePID(float error, float &integral, float &lastError,
                  float kp, float ki, float kd, float dt, float maxOutput) {
    integral += error * dt;
    integral = constrain(integral, -maxOutput, maxOutput); // anti-windup

    float derivative = (dt > 0) ? (error - lastError) / dt : 0;
    lastError = error;

    float output = (kp * error) + (ki * integral) + (kd * derivative);
    return constrain(output, -maxOutput, maxOutput);
}

// ============================================================
// DEPTH HOLD PID
// ============================================================
float computeDepthPID(
    float depth,
    float setpoint,
    float &integral,
    float &lastError,
    float dt
) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    // Positif jika robot lebih dangkal dari target.
    float error = setpoint - depth;

    // Deadband: ketika sudah sangat dekat dengan target, hentikan koreksi
    // dan lepaskan sebagian integral untuk mengurangi overshoot.
    if (fabs(error) <= DEPTH_DEADBAND) {
        integral *= 0.95f;
        lastError = error;
        return 0.0f;
    }

    integral += error * dt;
    integral = constrain(
        integral,
        -((float)MAX_DEPTH_CORRECTION),
        ((float)MAX_DEPTH_CORRECTION)
    );

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output =
        (DEPTH_KP * error) +
        (DEPTH_KI * integral) +
        (DEPTH_KD * derivative);

    output *= DEPTH_OUTPUT_SIGN;

    return constrain(
        output,
        -((float)MAX_DEPTH_CORRECTION),
        ((float)MAX_DEPTH_CORRECTION)
    );
}

// ════════════════════════════════════════════════════════════
//  MIXING KINEMATICS
// ════════════════════════════════════════════════════════════
// heaveCmd: nilai heave EFEKTIF (sudah melalui logika depth-hold, bukan cmd.heave mentah)
// pitchCorr & rollCorr: offset PWM dari stabilisasi IMU (background, selalu aktif)
ThrusterOutput mixing(const AutoCommand& cmd, int heaveCmd, float pitchCorr, float rollCorr) {
    ThrusterOutput out;
    int surge = constrain(cmd.surge - PWM_NEUTRAL, -400, 400);
    int yaw   = constrain(cmd.yaw   - PWM_NEUTRAL, -400, 400);
    int heave = constrain(heaveCmd  - PWM_NEUTRAL, -400, 400);
    
    // --- KOREKSI ROTASI FISIK 90 DERAJAT ---
    // Joystick Maju/Mundur (cmd.tilt) dialihkan menjadi perintah Roll, 
    // dengan nilai dibalik agar arahnya sesuai dengan fisik thruster depan/belakang
    int roll  = constrain(PWM_NEUTRAL - cmd.tilt, -400, 400); 
    
    // Joystick Kanan/Kiri (cmd.roll) dialihkan menjadi perintah Tilt
    int tilt  = constrain(cmd.roll - PWM_NEUTRAL, -400, 400); 

    // --- LOGIKA FILTER ISOLASI TILT & ROLL ---
    // Karena saat maju PWM berkurang (minus), tilt_depan menangkap nilai < 0
    int tilt_depan    = (tilt < 0) ? tilt : 0;
    // Sebaliknya, tilt_belakang menangkap nilai > 0
    int tilt_belakang = (tilt > 0) ? tilt : 0;
    
    // Logika Roll
    int roll_kanan = (roll > 0) ? roll : 0;
    int roll_kiri  = (roll < 0) ? roll : 0;

    // Thruster Horizontal (Surge & Yaw tidak berubah)
    out.TKIRI  = constrain(PWM_NEUTRAL + surge - yaw, PWM_MIN, PWM_MAX);
    out.TKANAN = constrain(PWM_NEUTRAL + surge + yaw, PWM_MIN, PWM_MAX);
    
    // Thruster Vertikal
    // DKIRI/DKANAN = pasangan depan, BKIRI/BKANAN = pasangan belakang
    // DKIRI/BKIRI  = pasangan kiri,  DKANAN/BKANAN = pasangan kanan
    // pitchCorr: + menambah dorongan depan & mengurangi belakang (mengoreksi kemiringan depan-belakang)
    // rollCorr : + menambah dorongan kiri & mengurangi kanan (mengoreksi kemiringan kiri-kanan)
    // CATATAN: tanda (+/-) korkeksi di atas adalah asumsi awal berdasarkan konvensi Mahony
    // (pitch/roll dari atan2 & asin di loop()). WAJIB diverifikasi saat uji hardware pertama;
    // jika arah koreksi terbalik (robot makin miring, bukan makin rata), balik tanda pitchCorr/rollCorr di sini.
    int pc = (int)pitchCorr;
    int rc = (int)rollCorr;

    out.DKIRI  = constrain(PWM_NEUTRAL - heave + roll_kiri  + tilt_depan    + rc + pc, PWM_MIN, PWM_MAX);
    out.DKANAN = constrain(PWM_NEUTRAL - heave - roll_kanan + tilt_depan    - rc + pc, PWM_MIN, PWM_MAX);
    out.BKIRI  = constrain(PWM_NEUTRAL - heave + roll_kiri  - tilt_belakang + rc - pc, PWM_MIN, PWM_MAX);
    out.BKANAN = constrain(PWM_NEUTRAL - heave - roll_kanan - tilt_belakang - rc - pc, PWM_MIN, PWM_MAX);
    
    return out;
}

// ════════════════════════════════════════════════════════════
//  APPLY OUTPUT (THRUSTER & SERVO)
// ════════════════════════════════════════════════════════════
void applyOutput(const ThrusterOutput& out, const AutoCommand& cmd) {
    if (cmd.mode == 3) {
        digitalWrite(PIN_RELAY, HIGH); 
        tDKIRI.write(PWM_NEUTRAL); tDKANAN.write(PWM_NEUTRAL);
        tBKIRI.write(PWM_NEUTRAL); tBKANAN.write(PWM_NEUTRAL);
        tTKIRI.write(PWM_NEUTRAL); tTKANAN.write(PWM_NEUTRAL);
    } else {
        digitalWrite(PIN_RELAY, LOW); 
        tDKIRI.write(out.DKIRI); tDKANAN.write(out.DKANAN);
        tBKIRI.write(out.BKIRI); tBKANAN.write(out.BKANAN);
        tTKIRI.write(out.TKIRI); tTKANAN.write(out.TKANAN);
    }
    
    // FIX 360: Pemetaan derajat ke sinyal mikrodetik (500-2500)
    servoTiltArm.writeMicroseconds(map(cmd.tiltArm, 0, 360, 500, 2500));
    servoGripper.writeMicroseconds(map(cmd.gripper, 0, 360, 500, 2500));
}

// ════════════════════════════════════════════════════════════
//  NON-BLOCKING SERIAL READ
// ════════════════════════════════════════════════════════════
void readCommand() {
    while (Serial.available() > 0 && newData == false) {
        char rc = Serial.read();
        static byte ndx = 0;
        if (rc != '\n') {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) ndx = numChars - 1; 
        } else {
            receivedChars[ndx] = '\0'; 
            ndx = 0;
            newData = true;
        }
    }

    if (newData) {
        newData = false; 
        if (receivedChars[0] == 'M' && receivedChars[1] == ':') {
            char* token = strtok(receivedChars + 2, ",");
            if (token) autoCmd.surge = constrain(atoi(token), PWM_MIN, PWM_MAX);
            token = strtok(NULL, ",");
            if (token) autoCmd.yaw = constrain(atoi(token), PWM_MIN, PWM_MAX);
            token = strtok(NULL, ",");
            if (token) autoCmd.roll = constrain(atoi(token), PWM_MIN, PWM_MAX);
            token = strtok(NULL, ",");
            if (token) autoCmd.tilt = constrain(atoi(token), PWM_MIN, PWM_MAX);
            token = strtok(NULL, ",");
            if (token) autoCmd.heave = constrain(atoi(token), PWM_MIN, PWM_MAX);
            token = strtok(NULL, ",");
            if (token) autoCmd.tiltArm = constrain(atoi(token), 0, 360); // FIX 360: max 360
            token = strtok(NULL, ",");
            if (token) autoCmd.gripper = constrain(atoi(token), 0, 360); // FIX 360: max 360
            token = strtok(NULL, ",");
            if (token) autoCmd.mode = atoi(token);
            token = strtok(NULL, ",");
            if (token) autoCmd.depthHold = atoi(token);
            token = strtok(NULL, ",");
            if (token) autoCmd.relevel = atoi(token);

            // --- LOGIKA ARMING ---
            if (autoCmd.mode == 1 || autoCmd.mode == 2) {
                systemArmed = true;
            }
            if (!systemArmed && autoCmd.mode == 3) {
                autoCmd.mode = 1;
            }
            // ---------------------

            lastAutoSerial = millis();
        }
    }
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW); 

    tDKIRI.attach(PIN_DKIRI, true);
    tDKANAN.attach(PIN_DKANAN, false);
    tBKIRI.attach(PIN_BKIRI, false);
    tBKANAN.attach(PIN_BKANAN, false);
    tTKIRI.attach(PIN_TKIRI, true);
    tTKANAN.attach(PIN_TKANAN, false);

    servoTiltArm.attach(PIN_TILT_ARM, 500, 2500);
    servoGripper.attach(PIN_GRIPPER,  500, 2500);
    
    // FIX 360: Set posisi awal ke titik tengah dengan writeMicroseconds
    servoTiltArm.writeMicroseconds(map(autoCmd.tiltArm, 0, 360, 500, 2500));
    servoGripper.writeMicroseconds(map(autoCmd.gripper, 0, 360, 500, 2500));

    Wire.begin();
    imu.begin();

    // Bypass I2C MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();
    delay(100);

    hmc5883.begin();

    depthSensor.init();
    depthSensor.setFluidDensity(997);   

    lastTime = micros(); 
    Serial.println("ROV Ready");
}

// ════════════════════════════════════════════════════════════
//  LOOP UTAMA
// ════════════════════════════════════════════════════════════
void loop() {
    // 1. Dengar perintah masuk setiap saat (tanpa blokir)
    readCommand();

    // 2. Eksekusi kontrol dengan rate konstan 50Hz (20ms)
    if (millis() - lastTaskTime >= taskInterval) {
        
        // ── Delta Time (Presisi Tinggi via micros) ──
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0f;
        lastTime = currentTime;

        // ── Sensor Read & Filter ──
        sensors_event_t a, g, temp, mag;
        imu.getEvent(&a, &g, &temp);
        hmc5883.getEvent(&mag);

        MahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z,
                     a.acceleration.x, a.acceleration.y, a.acceleration.z,
                     mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, dt);

        float roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG;
        float pitch = asin (2*(q0*q2 - q3*q1))                        * RAD_TO_DEG;
        float yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;
        if (yaw < 0) yaw += 360;

        depthSensor.read();
        float depth = depthSensor.depth();

        // ── Failsafe Timeout ──
        if (millis() - lastAutoSerial > SERIAL_TIMEOUT_MS) {
            int lastTiltArm = autoCmd.tiltArm;
            int lastGripper = autoCmd.gripper;
            autoCmd = AutoCommand(); 
            autoCmd.tiltArm = lastTiltArm; 
            autoCmd.gripper = lastGripper;

            // Reset Depth Hold saat komunikasi putus.
            prevDepthHoldFlag = false;
            depthIntegral = 0.0f;
            lastDepthError = 0.0f;
            depthError = 0.0f;
            depthCorrection = 0.0f;
            
            // --- LOGIKA ARMING ---
            if (systemArmed) {
                autoCmd.mode = 3; 
            } else {
                autoCmd.mode = 1;
            }
        }

        // ── Setpoint IMU Awal (Auto-Zero saat Boot) ──
        if (!attitudeSetpointInit) {
            pitchSetpoint = pitch;
            rollSetpoint  = roll;
            attitudeSetpointInit = true;
        }

        // ── Relevel (B3): re-zero setpoint pitch/roll ──
        bool relevelEdge = (autoCmd.relevel && !prevRelevelFlag);
        if (relevelEdge) {
            pitchSetpoint  = pitch;
            rollSetpoint   = roll;
            pitchIntegral  = 0; rollIntegral = 0;
            lastPitchError = 0; lastRollError = 0;
        }
        prevRelevelFlag = autoCmd.relevel;

        // ============================================================
        // DEPTH HOLD B0
        // ============================================================
        // B0 OFF : heave manual.
        // B0 ON  : lock depth saat B0 ditekan, lalu PID mempertahankan depth.
        // B0 OFF : PID di-reset dan kembali ke manual.

        bool depthHoldON = (autoCmd.depthHold && autoCmd.mode != 3);

        depthError = 0.0f;
        depthCorrection = 0.0f;

        // Rising edge: B0 baru ON -> simpan depth sekarang sebagai setpoint.
        bool depthHoldRisingEdge = depthHoldON && !prevDepthHoldFlag;
        if (depthHoldRisingEdge) {
            depthSetpoint = depth;
            depthIntegral = 0.0f;
            lastDepthError = 0.0f;

            Serial.print("DEPTH HOLD ON | SETPOINT = ");
            Serial.println(depthSetpoint, 3);
        }

        // Falling edge: B0 baru OFF -> reset PID.
        bool depthHoldFallingEdge = !depthHoldON && prevDepthHoldFlag;
        if (depthHoldFallingEdge) {
            depthIntegral = 0.0f;
            lastDepthError = 0.0f;
            depthError = 0.0f;
            depthCorrection = 0.0f;

            Serial.println("DEPTH HOLD OFF");
        }

        prevDepthHoldFlag = depthHoldON;

        // Heave efektif yang benar-benar masuk ke mixing.
        int effectiveHeave = autoCmd.heave;

        if (depthHoldON) {
            depthError = depthSetpoint - depth;

            depthCorrection = computeDepthPID(
                depth,
                depthSetpoint,
                depthIntegral,
                lastDepthError,
                dt
            );

            effectiveHeave = constrain(
                PWM_NEUTRAL + (int)depthCorrection,
                PWM_MIN,
                PWM_MAX
            );
        }

        // ── Stabilisasi IMU (Pitch/Roll) — background, selalu aktif, hanya utk 4 thruster vertikal ──
        float pitchError = pitchSetpoint - pitch;
        float rollError  = rollSetpoint  - roll;
        float pitchCorrection = computePID(pitchError, pitchIntegral, lastPitchError,
                                             ANGLE_KP, ANGLE_KI, ANGLE_KD, dt, MAX_ANGLE_CORRECTION);
        float rollCorrection  = computePID(rollError,  rollIntegral,  lastRollError,
                                             ANGLE_KP, ANGLE_KI, ANGLE_KD, dt, MAX_ANGLE_CORRECTION);
        if (autoCmd.mode == 3) {
            // Emergency: jangan berikan koreksi apapun
            pitchCorrection = 0;
            rollCorrection  = 0;
        }
        // Stabilisasi dinonaktifkan sementara jika ENABLE_IMU_STABILIZATION = false —
        // koreksi tetap dihitung & di-print untuk referensi tuning, tapi tidak
        // disalurkan ke mixing thruster.
        float mixingPitchCorr = ENABLE_IMU_STABILIZATION ? pitchCorrection : 0.0f;
        float mixingRollCorr  = ENABLE_IMU_STABILIZATION ? rollCorrection  : 0.0f;

        // ── Execute Motors ──
        ThrusterOutput out = mixing(autoCmd, effectiveHeave, mixingPitchCorr, mixingRollCorr);
        applyOutput(out, autoCmd);

        // ── Telemetry Out ──
        Serial.print("P:");  Serial.print(pitch, 1);
        Serial.print(" R:"); Serial.print(roll,  1);
        Serial.print(" Y:"); Serial.print(yaw,   1);
        Serial.print(" D:"); Serial.print(depth, 3);
        Serial.print(" DH:"); Serial.print(autoCmd.depthHold);
        Serial.print(" SPD:"); Serial.print(depthSetpoint, 3);
        Serial.print(" ERR:"); Serial.print(depthError, 3);
        Serial.print(" CORR:"); Serial.print(depthCorrection, 1);
        Serial.print(" HEFF:"); Serial.print(effectiveHeave);
        Serial.print(" IMU_STAB:"); Serial.print(ENABLE_IMU_STABILIZATION ? 1 : 0);
        Serial.print(" PC_raw:"); Serial.print(pitchCorrection, 1);
        Serial.print(" RC_raw:"); Serial.println(rollCorrection, 1);

        Serial.print("CMD S:");     Serial.print(autoCmd.surge);
        Serial.print(" Y:");        Serial.print(autoCmd.yaw);
        Serial.print(" H:");        Serial.print(autoCmd.heave);
        Serial.print(" R:");        Serial.print(autoCmd.roll);
        Serial.print(" T:");        Serial.print(autoCmd.tilt);
        Serial.print(" TiltArm:");  Serial.print(autoCmd.tiltArm);
        Serial.print(" Gripper:");  Serial.print(autoCmd.gripper);
        Serial.print(" Mode:");     Serial.print(autoCmd.mode);
        Serial.print(" DH_raw:");   Serial.print(autoCmd.depthHold);
        Serial.print(" RL_raw:");   Serial.println(autoCmd.relevel);

        Serial.print("PWM DKIRI:"); Serial.print(out.DKIRI);
        Serial.print(" DKANAN:");   Serial.print(out.DKANAN);
        Serial.print(" BKIRI:");    Serial.print(out.BKIRI);
        Serial.print(" BKANAN:");   Serial.print(out.BKANAN);
        Serial.print(" TKIRI:");    Serial.print(out.TKIRI);
        Serial.print(" TKANAN:");   Serial.println(out.TKANAN);

        lastTaskTime = millis();
    }
}