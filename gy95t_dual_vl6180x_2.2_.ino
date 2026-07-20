#include <Wire.h>
#include "VL6180X.h"

#define iic_add  0xa4 >> 1

// --- Accelerometer scale factor ---
// Register 0x07 bits[1:0] select the range; this module's factory default
// is ±16g (see GY-95T register map), and this code never changes it.
// If you ever write a different range to 0x07, update ACC_RANGE_G to match:
//   0: ±2g   1: ±4g   2: ±8g   3: ±16g  (default)
#define ACC_RANGE_G   16.0f
#define ACC_SCALE     (ACC_RANGE_G / 32768.0f)  // g per raw LSB

// Standard gravity, used only to convert the already-filtered g values to
// mm/s^2 right before printing. Filtering still happens in g's (that's the
// scale kf_acc_x/kf_acc_y's q/r were tuned against) - converting the output
// afterwards is mathematically identical to filtering in mm/s^2 directly
// (the Kalman filter is linear), so this does not change filter behavior.
#define ACC_G_TO_MMS2 9806.65f

// --- Dual VL6180X setup ---
// SHUT (shutdown) pin on each breakout must be wired to its own Arduino
// digital pin so we can bring the sensors up one at a time and reassign
// the second one's I2C address (both default to 0x29 out of the box, and
// two devices can't coexist on the bus at the same address).
#define SENSOR1_SHUT_PIN  4
#define SENSOR2_SHUT_PIN  5
#define SENSOR2_ADDRESS   0x30   // any free 7-bit I2C address != 0x29

typedef struct __attribute__((packed))
{
    int16_t acc_x;   
    int16_t acc_y;   
    int16_t acc_z;   
    int16_t gyro_x;  
    int16_t gyro_y;  
    int16_t gyro_z;  
    int16_t roll;    
    int16_t pitch;   
    int16_t yaw;     
    uint8_t leve;    
    int16_t temp;    
    int16_t mag_x;   
    int16_t mag_y;   
    int16_t mag_z;   
} gy;

gy my_95Q;
unsigned int delay_t = 0;
volatile byte ready_Ok = 0; // Added 'volatile' since it's changed in an ISR

VL6180X sensor1;
VL6180X sensor2;

// ---------------------------------------------------------------------
// Generic scalar Kalman filter (1D, constant-value model).
//
// This is the classic "predict/update" scalar Kalman filter:
//   - q = process noise covariance: how much we expect the true value to
//         drift between samples. Bigger q -> filter trusts new measurements
//         more and reacts faster (less smoothing).
//   - r = measurement noise covariance: how noisy the sensor is. Bigger r ->
//         filter trusts its own estimate more and smooths harder.
//   - x = current best estimate of the true value.
//   - p = current estimate error covariance (how uncertain we are about x).
//   - k = Kalman gain, recomputed every update.
//
// Starting p high means the very first measurement is trusted almost
// completely, so there's no "ramp up from zero" problem like the old EMA
// filter had - no separate "seed on first sample" flag is needed.
// ---------------------------------------------------------------------
typedef struct
{
    float q;
    float r;
    float x;
    float p;
    float k;
} KalmanFilter;

void kalmanInit(KalmanFilter *kf, float process_noise, float measurement_noise, float initial_p)
{
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->x = 0.0f;
    kf->p = initial_p; // large -> "I don't trust this initial value at all yet"
}

float kalmanUpdate(KalmanFilter *kf, float measurement)
{
    // --- Predict ---
    // Constant-value model: our best guess of the next state is the same
    // as the current one, but we're a little less sure of it each step.
    kf->p = kf->p + kf->q;

    // --- Update (correct prediction using the new measurement) ---
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}

// --- Accelerometer Kalman filter state ---
// NOTE: q/r are tuned for values in g's (typically 0-16 range), not
// raw counts (0-32768 range) and not mm/s^2. The filtered g value is
// converted to mm/s^2 only at print time - see ACC_G_TO_MMS2 above.
// Tune q up if the filter feels laggy/sluggish behind real motion.
// Tune r up if the output still looks jittery/noisy.
KalmanFilter kf_acc_x;
KalmanFilter kf_acc_y;

// --- Distance filter state ---
// One filter per sensor - noise is independent per device, so sharing a
// single filter between two sensors would let one sensor's readings bleed
// into the other's estimate.
KalmanFilter kf_dist;   // sensor1
KalmanFilter kf_dist2;  // sensor2

void setup() {
    Serial.begin(9600); // <-- CRANK THIS UP to prevent serial choking
    delay(500); // give the serial monitor time to connect on boards that reset on open

    Wire.begin();
    delay(100);
    
    // --- GY95T setup ---
    byte td = 1;
    bool td_ok = iic_read(0x02, &td, 1);
    switch (td)
    {
        case 0: delay_t = 100; break;
        case 1: delay_t = 20;  break;
        case 2: delay_t = 10;  break;
        case 3: delay_t = 5;   break;
    }
    delay(delay_t);
    
    // Interrupt 0 is Pin 2 on Uno/Nano
    attachInterrupt(0, Exti, RISING); 
    
    // --- VL6180X setup (two sensors, sequential address assignment) ---
    pinMode(SENSOR1_SHUT_PIN, OUTPUT);
    pinMode(SENSOR2_SHUT_PIN, OUTPUT);

    // Hold both sensors in shutdown so neither answers on the bus yet.
    digitalWrite(SENSOR1_SHUT_PIN, LOW);
    digitalWrite(SENSOR2_SHUT_PIN, LOW);
    delay(10);

    // Bring sensor2 up FIRST, while sensor1 is still held in shutdown.
    // This way sensor2 is the only device on the bus while it's sitting
    // at the power-on default address (0x29), and we can safely move it
    // off to SENSOR2_ADDRESS before sensor1 ever powers up. If we brought
    // sensor1 up first instead, there would be a window where both chips
    // are simultaneously live at 0x29 -> bus contention / possible lockup.
    digitalWrite(SENSOR2_SHUT_PIN, HIGH);
    delay(10);
    sensor2.init();
    sensor2.configureDefault();
    sensor2.setAddress(SENSOR2_ADDRESS);
    sensor2.setTimeout(500);

    // Now bring sensor1 up. sensor2 has already vacated 0x29, so sensor1
    // can safely stay at its power-on default address.
    digitalWrite(SENSOR1_SHUT_PIN, HIGH);
    delay(10);
    sensor1.init();
    sensor1.configureDefault();
    sensor1.setTimeout(500);

    // --- Kalman filter init ---
    // Accelerometer: filtered in g's (±16g range -> LSB = 0.000488g), then
    // converted to mm/s^2 for printing.
    // Starting point: q=0.02 (allow ~0.02g drift/sample - responsive to real
    // motion), r=0.05 (assume ~0.05g measurement noise). Retune by watching
    // the printed output: laggy -> raise q; jittery -> raise r.
    kalmanInit(&kf_acc_x, 0.02f, 0.05f, 10.0f);
    kalmanInit(&kf_acc_y, 0.02f, 0.05f, 10.0f);

    // Distance: VL6180X millimeter readings are usually stable when valid,
    // so a smaller q gives heavier smoothing / higher precision on distance.
    kalmanInit(&kf_dist, 0.5f, 9.0f, 1000.0f);
    kalmanInit(&kf_dist2, 0.5f, 9.0f, 1000.0f);
}

void loop() {
    // Only process when the GY95T interrupt signals new data is ready
    if (ready_Ok) {
        ready_Ok = 0; // Clear the flag immediately
        
        unsigned char data[27] = {0};

        // --- Read GY95T (accel/gyro/mag) ---
        bool gy95t_ok = iic_read(0x08, data, 27);
        if (gy95t_ok) {
            memcpy(&my_95Q, data, 27);
        }

        // --- Read both VL6180X sensors (distance) ---
        int dist_raw_1 = sensor1.readRangeSingleMillimeters();
        bool dist_ok_1 = !sensor1.timeoutOccurred() && dist_raw_1 != 255;

        int dist_raw_2 = sensor2.readRangeSingleMillimeters();
        bool dist_ok_2 = !sensor2.timeoutOccurred() && dist_raw_2 != 255;

        if (!gy95t_ok || !dist_ok_1 || !dist_ok_2) {
            return; // something failed - skip the data line this frame
        }

        // --- Convert raw accel counts to g's, then Kalman filter (in g's) ---
        float acc_x_g = my_95Q.acc_x * ACC_SCALE;
        float acc_y_g = my_95Q.acc_y * ACC_SCALE;
        float acc_z_g = my_95Q.acc_z * ACC_SCALE; // unfiltered - stable already

        float acc_x_filt_g = kalmanUpdate(&kf_acc_x, acc_x_g);
        float acc_y_filt_g = kalmanUpdate(&kf_acc_y, acc_y_g);

        // --- Convert raw + filtered g values to mm/s^2 for output ---
        // acc_x_raw/acc_y_raw are printed alongside the filtered values
        // (same pattern as distance_mm_raw/distance_mm2_raw below) purely
        // so you can visually confirm on the serial plotter that the
        // filtered trace is actually smoother than the raw trace. Remove
        // once you've confirmed the filter is doing its job.
        float acc_x_raw_mms2 = acc_x_g * ACC_G_TO_MMS2;
        float acc_y_raw_mms2 = acc_y_g * ACC_G_TO_MMS2;
        float acc_x_mms2 = acc_x_filt_g * ACC_G_TO_MMS2;
        float acc_y_mms2 = acc_y_filt_g * ACC_G_TO_MMS2;
        float acc_z_mms2 = acc_z_g * ACC_G_TO_MMS2;

        // --- Distance: Kalman filter smooths each raw reading directly ---
        float dist_kalman_1 = kalmanUpdate(&kf_dist, dist_raw_1);
        float dist_kalman_2 = kalmanUpdate(&kf_dist2, dist_raw_2);

        // --- Print filtered values ---
        // Field names below match what gy95t_dual_vl6180x_slicer_cube.py
        // parses: acc_x/y/z (mm/s^2), roll, pitch, yaw, temp, leve,
        // mag_x/y/z, distance_mm (sensor1, Kalman-smoothed), distance_mm2
        // (sensor2, Kalman-smoothed). The _raw fields are extra, for
        // debugging only - the Slicer script ignores them.
        Serial.print("acc_x: ");  Serial.print(acc_x_mms2, 2);
        Serial.print(",acc_x_raw: "); Serial.print(acc_x_raw_mms2, 2);
        Serial.print(",acc_y: "); Serial.print(acc_y_mms2, 2);
        Serial.print(",acc_y_raw: "); Serial.print(acc_y_raw_mms2, 2);
        Serial.print(",acc_z: "); Serial.print(acc_z_mms2, 2);

        Serial.print(",roll: ");  Serial.print(my_95Q.roll / 100.0);
        Serial.print(",pitch: "); Serial.print(my_95Q.pitch / 100.0);
        Serial.print(",yaw: ");   Serial.print(my_95Q.yaw / 100.0);

        Serial.print(",temp: ");  Serial.print(my_95Q.temp / 100.0);
        Serial.print(",leve: ");  Serial.print((float)my_95Q.leve);

        Serial.print(",mag_x: "); Serial.print(my_95Q.mag_x);
        Serial.print(",mag_y: "); Serial.print(my_95Q.mag_y);
        Serial.print(",mag_z: "); Serial.print(my_95Q.mag_z);

        Serial.print(",distance_mm_raw: ");  Serial.print(dist_raw_1);
        Serial.print(",distance_mm: ");      Serial.print(dist_kalman_1);

        Serial.print(",distance_mm2_raw: "); Serial.print(dist_raw_2);
        Serial.print(",distance_mm2: ");     Serial.println(dist_kalman_2);
    }
}

void Exti()
{
    ready_Ok = 1; // Simplest possible ISR execution
}

bool iic_read(unsigned char add, unsigned char *data, unsigned char len)
{
    Wire.beginTransmission(iic_add);
    Wire.write(add);
    if (Wire.endTransmission(false) != 0) return false; // bus error

    byte received = Wire.requestFrom(iic_add, (int)len);
    if (received != len) return false; // incomplete read

    for (byte j = 0; j < len; j++) *data++ = Wire.read();
    return true;
}

void iic_write(char add, unsigned char data)
{
    Wire.beginTransmission(iic_add);
    Wire.write((uint8_t)add);
    Wire.write((uint8_t)data);
    Wire.endTransmission();
}