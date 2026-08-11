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
//  MIXING KINEMATICS
// ════════════════════════════════════════════════════════════
ThrusterOutput mixing(const AutoCommand& cmd) {
    ThrusterOutput out;
    int surge = constrain(cmd.surge - PWM_NEUTRAL, -400, 400);
    int yaw   = constrain(cmd.yaw   - PWM_NEUTRAL, -400, 400);
    int heave = constrain(cmd.heave - PWM_NEUTRAL, -400, 400);
    
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
    out.DKIRI  = constrain(PWM_NEUTRAL - heave + roll_kiri  + tilt_depan, PWM_MIN, PWM_MAX);
    out.DKANAN = constrain(PWM_NEUTRAL - heave - roll_kanan + tilt_depan, PWM_MIN, PWM_MAX);
    out.BKIRI  = constrain(PWM_NEUTRAL - heave + roll_kiri  - tilt_belakang, PWM_MIN, PWM_MAX);
    out.BKANAN = constrain(PWM_NEUTRAL - heave - roll_kanan - tilt_belakang, PWM_MIN, PWM_MAX);
    
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

    tDKIRI.attach(PIN_DKIRI, false);
    tDKANAN.attach(PIN_DKANAN, true);
    tBKIRI.attach(PIN_BKIRI, false);
    tBKANAN.attach(PIN_BKANAN, true);
    tTKIRI.attach(PIN_TKIRI, true);
    tTKANAN.attach(PIN_TKANAN, false);

    servoTiltArm.attach(PIN_TILT_ARM);
    servoGripper.attach(PIN_GRIPPER);
    
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
            
            // --- LOGIKA ARMING ---
            if (systemArmed) {
                autoCmd.mode = 3; 
            } else {
                autoCmd.mode = 1;
            }
        }

        // ── Execute Motors ──
        ThrusterOutput out = mixing(autoCmd);
        applyOutput(out, autoCmd);

        // ── Telemetry Out ──
        Serial.print("P:");  Serial.print(pitch, 1);
        Serial.print(" R:"); Serial.print(roll,  1);
        Serial.print(" Y:"); Serial.print(yaw,   1);
        Serial.print(" D:"); Serial.println(depth, 3);

        Serial.print("CMD S:");     Serial.print(autoCmd.surge);
        Serial.print(" Y:");        Serial.print(autoCmd.yaw);
        Serial.print(" H:");        Serial.print(autoCmd.heave);
        Serial.print(" R:");        Serial.print(autoCmd.roll);
        Serial.print(" T:");        Serial.print(autoCmd.tilt);
        Serial.print(" TiltArm:");  Serial.print(autoCmd.tiltArm);
        Serial.print(" Gripper:");  Serial.println(autoCmd.gripper);

        Serial.print("PWM DKIRI:"); Serial.print(out.DKIRI);
        Serial.print(" DKANAN:");   Serial.print(out.DKANAN);
        Serial.print(" BKIRI:");    Serial.print(out.BKIRI);
        Serial.print(" BKANAN:");   Serial.print(out.BKANAN);
        Serial.print(" TKIRI:");    Serial.print(out.TKIRI);
        Serial.print(" TKANAN:");   Serial.println(out.TKANAN);

        lastTaskTime = millis();
    }
}
