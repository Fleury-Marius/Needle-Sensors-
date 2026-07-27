/* =====================================================================
 * GY-95T + dual VL6180X -> 3D Slicer
 * Orientation, dead-reckoned position, and two live range probes.
 * Target: Arduino UNO (ATmega328P)
 * ---------------------------------------------------------------------
 * This is yesterday's IMU sketch with the distance sensors merged back
 * in. Three things to know before reading further:
 *
 * 1. THE RANGE READS ARE NON-BLOCKING NOW. Your original sketch called
 *    readRangeSingleMillimeters() on both sensors inside the sample
 *    loop. That call starts a measurement and then sits there polling
 *    until it finishes - roughly 10 ms per sensor, so ~20 ms of dead
 *    time every pass. That was tolerable when the sketch only printed
 *    angles. It is not tolerable now: position integration needs a
 *    steady dt at ~50 Hz, and 20 ms of blocking per pass wrecks it.
 *    Both sensors now run in continuous mode and we simply check "is a
 *    new sample sitting in the register?" each pass. If yes we take it,
 *    if no we move on. The IMU loop never waits for the rangefinders.
 *
 * 2. FILTERING. You mentioned you only had Kalman filters on acc_x/y.
 *    Filters are now on acc x/y/z, gyro x/y/z, and both range channels
 *    (kf_dist, kf_dist2), all as display/output smoothing.
 *
 * 3. RANGE-AIDED POSITION, now ON by default (USE_RANGE_CORRECTION).
 *    This is the part that turns dead reckoning into a real
 *    measurement - read the assumptions before trusting it.
 * ===================================================================== */

#include <Wire.h>
#include <VL6180X.h>
#include <math.h>

#define iic_add  (0xa4 >> 1)

/* ---------------- User-tunable ---------------- */

/* --- SERIAL BUDGET, and why the output format looks like this --------
 * ArduinoConnect tops out at 9600 baud = 960 bytes per second. The old
 * verbose line was ~500 bytes, so it could only be sent about twice a
 * second before Serial.print() started blocking the integration loop.
 *
 * So the format is now compact and TIERED. Every line carries the six
 * numbers the visualization actually needs to move the model - angles
 * and position - plus one rotating group of secondary values. Over five
 * lines you still get everything; each slow value just refreshes at a
 * fifth of the rate. The Slicer script merges the fields as they arrive,
 * so the corner box shows all of them and nothing reads as zero.
 *
 * Budget: core ~51 bytes + extras ~25 = ~78 bytes per line.
 *   8 Hz -> ~620 B/s, 65% of the link. Comfortable.
 *  10 Hz -> ~780 B/s, 81%. Works, less margin for the startup messages.
 * Do not go above 10 at 9600. If you ever get a faster link, raise
 * PRINT_HZ to 20 and the same format streams happily.
 * ------------------------------------------------------------------- */
#define SERIAL_BAUD       9600
#define PRINT_HZ          8

#define VERBOSE_STARTUP   1        // 0 = data lines ONLY (use this for Slicer)
// All secondary values now ride the rotating extras groups, so the old
// per-field SEND_* toggles are gone - there is no longer a per-line cost
// to any one of them.

#define I2C_CLOCK_HZ      100000L
#define USE_POLL_FALLBACK 1

/* --- VL6180X wiring ---------------------------------------------------
 * Both breakouts power up at address 0x29, so they cannot share the bus
 * until one is renamed. Each SHUT pin must go to its own Arduino pin so
 * we can hold one in reset while addressing the other. */
#define SENSOR1_SHUT_PIN  4
#define SENSOR2_SHUT_PIN  5
#define SENSOR3_SHUT_PIN  6
#define SENSOR2_ADDRESS   0x30     // any free 7-bit address that is not 0x29
#define SENSOR3_ADDRESS   0x31     // third sensor, another free address
#define RANGE_PERIOD_MS   50       // continuous-mode period, multiple of 10

// VL6180X range scaling. The part is designed for short range and this is
// the lever that trades resolution for reach:
//   1 -> 1 mm resolution, usable to roughly 200 mm  (default)
//   2 -> 2 mm resolution, usable to roughly 400 mm
//   3 -> 3 mm resolution, usable to roughly 600 mm
// Scaling 1 gives the best correction quality but means the needle has to
// stay within ~20 cm of the target or the correction simply stops. If you
// keep losing lock because you work further out, try 2. Update
// RANGE_MEAS_VAR below if you do - coarser scaling is noisier.
#define RANGE_SCALING     1
#define RANGE_MAX_VALID   (200 * RANGE_SCALING)   // mm

/* --- IMU scaling --- */
#define ACC_RANGE_G     16.0f
#define ACC_SCALE       (ACC_RANGE_G / 32768.0f)
#define ACC_G_TO_MMS2   9806.65f
#define GYRO_RANGE_DPS  2000.0f
#define GYRO_SCALE      (GYRO_RANGE_DPS / 32768.0f)
#define GRAVITY_SIGN    1.0f

/* --- ACCELEROMETER SCALE CORRECTION -----------------------------------
 * ACC_RANGE_G assumes the part is exactly +/-16g. It never is. Your own
 * captures showed roughly 9844 and 9867 mm/s^2 on Z while at rest, where
 * true gravity is 9806.65 - a scale error of about half a percent.
 *
 * That matters far more than it looks, and it hurts VERTICAL worst. An
 * additive bias calibration cannot remove it, because the error is
 * PROPORTIONAL to the reading: calibrate it away lying flat and it comes
 * straight back when you tilt, since the axis carrying gravity changed.
 * Half a percent of 9806 is ~50 mm/s^2 of phantom acceleration, which
 * double-integrates to 25 mm in one second and 2.5 m in ten.
 *
 * So we measure the true scale at startup instead of trusting the
 * datasheet: at rest the acceleration magnitude must be exactly 1g, and
 * whatever factor makes that true is the correction.
 * ------------------------------------------------------------------- */
#define ACC_GAIN_MIN    0.95f
#define ACC_GAIN_MAX    1.05f

/* --- Position filter --- */
#define Q_ACCEL_VAR     4000.0f
#define VEL_TAU_S       4.0f
#define STAT_HOLD_MS    300UL
#define ZERO_YAW_AT_START 1
#define CAL_SAMPLES     200
#define STAT_WIN        8
#define BIAS_TRACK_A    0.02f
#define DIAG_PERIOD_MS  2000UL

/* --- RANGE-AIDED POSITION ---------------------------------------------
 * Set to 1 to let the rangefinders correct the position estimate.
 *
 * WHAT IT ASSUMES: the sensors are pointing at a flat surface that does
 * not move, roughly square-on. Under that assumption the surface point
 * is fixed in world space, so (position along the beam) + (measured
 * range) is a constant. Measure the range, and you have measured your
 * own position along that direction - a real observation, not an
 * integration. That is what bounds drift properly.
 *
 * WHEN IT WILL HURT YOU: if the target moves, if you sweep off the edge
 * of it, if you turn away so the beam hits something further off, or if
 * you approach at a steep angle. Then the "fixed surface" premise is
 * false and the correction will confidently drag position to the wrong
 * place. RANGE_INNOV_MAX guards against the worst of it by refusing
 * corrections that disagree violently with the IMU, but the guard is not
 * a substitute for pointing the thing at a real, static, flat target.
 *
 * The measurement observes position PROJECTED onto the beam, so the
 * update is spread across all three axes weighted by the beam direction.
 * It works at any orientation - there is no alignment threshold.
 *
 * Watch the two range lines in the Slicer box (one per sensor) to see
 * whether each is correcting: 0 no target, 1 waiting to anchor,
 * 2 locked, 3 rejecting.
 * ------------------------------------------------------------------- */
#define USE_RANGE_CORRECTION 1
#define RANGE_MEAS_VAR    36.0f    // (mm)^2; VL6180X is roughly +/-5 mm

// Reject any correction that disagrees with the current estimate by more
// than this. Sweeping off the edge of the target produces a sudden step
// of tens of mm - without this gate that step gets believed and yanks
// position with it.
#define RANGE_INNOV_MAX   40.0f    // mm

// A hand crossing the beam looks exactly like sudden self-motion to the
// anchor. These two guards tell them apart:
//
// RANGE_JUMP_MAX: the biggest believable change between consecutive
// readings. Real hand/needle motion is smooth at 8-50 Hz; a hand
// arriving in the beam is a step. Any single-sample jump larger than
// this is a target change, not motion - freeze the correction and let
// the reading re-settle before trusting it again.
#define RANGE_JUMP_MAX    12.0f    // mm per sample

// When the IMU says we are stationary, we KNOW the true innovation is
// zero, so any nonzero range change must be the target moving (a hand).
// While stationary we therefore do not correct at all - ZUPT already
// holds position - and we re-baseline the anchor to the current reading
// so that whatever the sensor sees at rest becomes the new zero. This is
// what stops "block a sensor -> needle slides toward it".
#define RANGE_TRUST_WHILE_MOVING_ONLY 1

// A range change smaller than this per sample, while the IMU is also
// stationary, counts as "not really moving" and re-baselines the anchor.
// Anything larger is treated as real motion and gets corrected even if
// the accelerometer is too insensitive to notice a slow creep. This is
// the knob that decides how slow a motion the sensors will still track:
// lower = tracks slower motion but re-baselines less aggressively at
// rest; higher = holds a firmer zero at rest but ignores very slow
// creeps. 1.5 mm at 8 Hz corresponds to about 12 mm/s.
#define RANGE_CREEP_MM    1.5f


#define RANGE_REJECT_MAX  15       // consecutive rejects before re-anchoring
#define RANGE_STABLE_MM   3.0f     // reading must be this steady to anchor

// How long the reading must stay steady before anchoring. Anchoring no
// longer waits for ZUPT: if the stationary detector never fires, the old
// code would never anchor, so the correction silently never ran.
#define RANGE_ANCHOR_MS   800UL

/* --- BEAM DIRECTIONS, one per sensor, in the BODY frame ---------------
 * NEW LAYOUT: the two sensors now point along different body axes, so
 * they constrain two independent directions instead of one. A single
 * rangefinder only ever pins the direction it looks along; two
 * perpendicular ones pin two.
 *
 *   Sensor 1 -> body +X  ("right")
 *   Sensor 2 -> body +Y  ("forward")
 *
 * "Body frame" means the IMU's own axes, which rotate with the syringe.
 * At startup, with ZERO_YAW_AT_START on, the world X/Y axes are aligned
 * to wherever the device is pointing then - so body +X maps to world +X
 * and body +Y to world +Y while the device is roughly level and hasn't
 * been turned. Tilt or yaw it and the code re-projects both beams into
 * the world frame every sample, so the correction always lands on the
 * right world axis regardless of pose.
 *
 * If you physically mount them the other way round (sensor 1 forward,
 * sensor 2 right), just swap the two blocks below - nothing else changes.
 * These must stay in sync with SENSOR1/2_LOCAL_DIR in the Slicer script.
 * ------------------------------------------------------------------- */
#define BEAM1_BODY_X  -1.0f     // sensor 1 -> -X (points left, per hardware)
#define BEAM1_BODY_Y  0.0f
#define BEAM1_BODY_Z  0.0f

#define BEAM2_BODY_X  0.0f
#define BEAM2_BODY_Y  1.0f      // sensor 2 -> +Y (forward)
#define BEAM2_BODY_Z  0.0f

#define BEAM3_BODY_X  0.0f
#define BEAM3_BODY_Y  0.0f
#define BEAM3_BODY_Z  1.0f      // sensor 3 -> +Z (up), the vertical channel


/* ---------------------------------------------------------------------
 * Typedefs stay at the top: the Arduino IDE injects auto-generated
 * prototypes near the top of the file, and any type used in a function
 * signature must be visible before them.
 * ------------------------------------------------------------------- */

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

typedef struct
{
    float q;
    float r;
    float x;
    float p;
    float k;
} KalmanFilter;

typedef struct
{
    float pos;
    float vel;
    float P[2][2];
} PosVelKF;

// One range-correction channel per sensor. Declared up here with the
// other typedefs, NOT down beside the correction code, because the
// Arduino IDE auto-generates function prototypes near the top of the
// file - any type used in a signature (updateRangeChannel takes a
// RangeChannel*) must be visible before those prototypes, or you get
// "variable or field declared void" / "not declared in this scope".
// state, published per channel to Slicer:
//   0 no usable reading   1 waiting to anchor
//   2 locked, correcting  3 reading rejected as an outlier
typedef struct
{
    float bx, by, bz;              // beam direction in the body frame
    bool  horizontal_only;         // true = flatten to XY; false = keep Z
    bool  anchored;
    float anchor_C;                // (position along beam + range) at anchor
    float last_mm;
    float prev_mm;                 // reading one sample ago, for jump test
    float innov;                   // measurement minus estimate, mm
    unsigned long stable_since_ms;
    uint8_t reject_count;
    uint8_t state;
} RangeChannel;

gy my_95Q;
VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;

unsigned int delay_t = 20;
volatile byte ready_Ok = 0;
volatile unsigned long int_count = 0;

unsigned long i2c_fail_count = 0;
unsigned long sample_count = 0;
unsigned long last_sample_ms = 0;
unsigned long last_diag_ms = 0;
unsigned long last_update_us = 0;
unsigned long last_print_ms = 0;
uint8_t extras_group = 0;        // which secondary group goes out this line

// Latest accepted range readings, -1 when we have never had a valid one.
float dist1_mm = -1.0f;
float dist2_mm = -1.0f;
float dist3_mm = -1.0f;
int   dist1_raw = -1;
int   dist2_raw = -1;
bool  sensor1_present = false;
bool  sensor2_present = false;
bool  sensor3_present = false;

/* ---------------- Scalar Kalman (output smoothing) ------------------ */

void kalmanInit(KalmanFilter *kf, float process_noise, float measurement_noise, float initial_p)
{
    kf->q = process_noise;
    kf->r = measurement_noise;
    kf->x = 0.0f;
    kf->p = initial_p;
}

float kalmanUpdate(KalmanFilter *kf, float measurement)
{
    kf->p = kf->p + kf->q;
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;
    return kf->x;
}

KalmanFilter kf_acc_x, kf_acc_y, kf_acc_z;
KalmanFilter kf_gyro_x, kf_gyro_y, kf_gyro_z;
KalmanFilter kf_dist, kf_dist2, kf_dist3;

/* ---------------- Position / velocity filter ------------------------ */

void posvelInit(PosVelKF *kf, float pos0, float vel0, float pos_var0, float vel_var0)
{
    kf->pos = pos0;
    kf->vel = vel0;
    kf->P[0][0] = pos_var0; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;     kf->P[1][1] = vel_var0;
}

void posvelPredict(PosVelKF *kf, float accel, float dt, float q_accel_var)
{
    kf->pos += kf->vel * dt + 0.5f * accel * dt * dt;
    kf->vel += accel * dt;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    float Qpp = 0.25f * dt4 * q_accel_var;
    float Qpv = 0.5f  * dt3 * q_accel_var;
    float Qvv = dt2 * q_accel_var;

    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    float P10 = kf->P[1][0], P11 = kf->P[1][1];

    kf->P[0][0] = P00 + dt * (P01 + P10) + dt2 * P11 + Qpp;
    kf->P[0][1] = P01 + dt * P11 + Qpv;
    kf->P[1][0] = P10 + dt * P11 + Qpv;
    kf->P[1][1] = P11 + Qvv;
}

// THE STEP THAT WAS MISSING WITHOUT THE DISTANCE SENSORS.
// Standard scalar measurement update for the 2-state filter, H = [1 0]:
// an observation of position also corrects velocity, through the
// position/velocity covariance the predict step has been building up.
void posvelCorrect(PosVelKF *kf, float z_pos, float meas_var)
{
    float S  = kf->P[0][0] + meas_var;
    if (S <= 0.0f) return;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    float innov = z_pos - kf->pos;
    kf->pos += K0 * innov;
    kf->vel += K1 * innov;

    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    float P10 = kf->P[1][0], P11 = kf->P[1][1];

    kf->P[0][0] = P00 - K0 * P00;
    kf->P[0][1] = P01 - K0 * P01;
    kf->P[1][0] = P10 - K1 * P00;
    kf->P[1][1] = P11 - K1 * P01;
}

void posvelZUPT(PosVelKF *kf, float vel_var_after_zupt)
{
    kf->vel = 0.0f;
    kf->P[1][1] = vel_var_after_zupt;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
}

void posvelLeak(PosVelKF *kf, float leak)
{
    kf->vel *= leak;
}

PosVelKF kf_pos_x, kf_pos_y, kf_pos_z;

/* ---------------- Stationary detector ------------------------------- */

float acc_hist[STAT_WIN];
float gyr_hist[STAT_WIN];
uint8_t hist_i = 0;
uint8_t hist_n = 0;

float tol_acc_offset = 0.02f;
float tol_acc_std    = 0.01f;
float tol_gyro       = 5.0f;

void statPush(float acc_mag_g, float gyro_mag_dps)
{
    acc_hist[hist_i] = acc_mag_g;
    gyr_hist[hist_i] = gyro_mag_dps;
    hist_i = (hist_i + 1) % STAT_WIN;
    if (hist_n < STAT_WIN) hist_n++;
}

bool statIsStationary()
{
    if (hist_n < STAT_WIN) return false;

    float mean_a = 0.0f, mean_g = 0.0f;
    for (uint8_t i = 0; i < STAT_WIN; i++) { mean_a += acc_hist[i]; mean_g += gyr_hist[i]; }
    mean_a /= STAT_WIN;
    mean_g /= STAT_WIN;

    float var_a = 0.0f;
    for (uint8_t i = 0; i < STAT_WIN; i++) {
        float d = acc_hist[i] - mean_a;
        var_a += d * d;
    }
    var_a /= STAT_WIN;

    return (fabs(mean_a - 1.0f) < tol_acc_offset)
        && (sqrt(var_a)        < tol_acc_std)
        && (mean_g             < tol_gyro);
}

/* ---------------- World frame --------------------------------------- */

float acc_bias_world_x = 0.0f;
float acc_bias_world_y = 0.0f;
float acc_bias_world_z = 0.0f;

// Multiplicative correction on every accel reading, measured at startup.
float acc_gain = 1.0f;

float yaw_ref_deg = 0.0f;
unsigned long stationary_since_ms = 0;

// Instances live here (after the BEAM_* macros they initialise from).
RangeChannel ch1 = {BEAM1_BODY_X, BEAM1_BODY_Y, BEAM1_BODY_Z,
                    true,  false, 0.0f, -1.0f, -1.0f, 0.0f, 0, 0, 0};
RangeChannel ch2 = {BEAM2_BODY_X, BEAM2_BODY_Y, BEAM2_BODY_Z,
                    true,  false, 0.0f, -1.0f, -1.0f, 0.0f, 0, 0, 0};
RangeChannel ch3 = {BEAM3_BODY_X, BEAM3_BODY_Y, BEAM3_BODY_Z,
                    false, false, 0.0f, -1.0f, -1.0f, 0.0f, 0, 0, 0};

float referencedYaw()
{
#if ZERO_YAW_AT_START
    return (my_95Q.yaw / 100.0f) - yaw_ref_deg;
#else
    return my_95Q.yaw / 100.0f;
#endif
}

void rotateBodyToWorld(float ax, float ay, float az,
                       float roll_deg, float pitch_deg, float yaw_deg,
                       float *wx, float *wy, float *wz)
{
    float roll  = roll_deg  * 0.017453293f;
    float pitch = pitch_deg * 0.017453293f;
    float yaw   = yaw_deg   * 0.017453293f;

    float cr = cos(roll),  sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw),   sy = sin(yaw);

    *wx = (cy*cp)*ax + (cy*sp*sr - sy*cr)*ay + (cy*sp*cr + sy*sr)*az;
    *wy = (sy*cp)*ax + (sy*sp*sr + cy*cr)*ay + (sy*sp*cr - cy*sr)*az;
    *wz = (-sp)*ax   + (cp*sr)*ay            + (cp*cr)*az;
}

/* ---------------- VL6180X, non-blocking ------------------------------
 * Register numbers are written as literals rather than library enum
 * names so this compiles against any version of the Pololu library.
 *   0x4F  RESULT__INTERRUPT_STATUS_GPIO
 *   0x4D  RESULT__RANGE_STATUS
 *   0x62  RESULT__RANGE_VAL
 *   0x15  SYSTEM__INTERRUPT_CLEAR
 * ------------------------------------------------------------------- */

// Returns true and fills out_mm only when a fresh sample is waiting.
// Never waits for a conversion to finish.
bool rangePoll(VL6180X &s, int *out_mm)
{
    uint8_t status = s.readReg(0x4F);
    if ((status & 0x07) != 0x04) return false;      // no new sample yet

    uint8_t err = s.readReg(0x4D) >> 4;             // 0 = valid measurement
    uint8_t mm  = s.readReg(0x62);
    s.writeReg(0x15, 0x01);                         // clear the interrupt

    if (err != 0 || mm == 255 || mm > RANGE_MAX_VALID) {
        *out_mm = -1;
        return true;                                // fresh, but unusable
    }
    *out_mm = (int)mm;
    return true;
}

void serviceRangeSensors()
{
    int mm;
    if (sensor1_present && rangePoll(sensor1, &mm)) {
        dist1_raw = mm;
        dist1_mm  = (mm < 0) ? -1.0f : kalmanUpdate(&kf_dist, (float)mm);
    }
    if (sensor2_present && rangePoll(sensor2, &mm)) {
        dist2_raw = mm;
        dist2_mm  = (mm < 0) ? -1.0f : kalmanUpdate(&kf_dist2, (float)mm);
    }
    if (sensor3_present && rangePoll(sensor3, &mm)) {
        dist3_mm  = (mm < 0) ? -1.0f : kalmanUpdate(&kf_dist3, (float)mm);
    }
}

void setupRangeSensors()
{
    pinMode(SENSOR1_SHUT_PIN, OUTPUT);
    pinMode(SENSOR2_SHUT_PIN, OUTPUT);
    pinMode(SENSOR3_SHUT_PIN, OUTPUT);

    // All three power up at 0x29, so only one may be awake at a time until
    // it has been given a unique address. Hold all in reset, then bring
    // them up one by one: sensor 3 first (gets 0x31), then sensor 2 (0x30),
    // then sensor 1 last (keeps the default 0x29).
    digitalWrite(SENSOR1_SHUT_PIN, LOW);
    digitalWrite(SENSOR2_SHUT_PIN, LOW);
    digitalWrite(SENSOR3_SHUT_PIN, LOW);
    delay(20);

    digitalWrite(SENSOR3_SHUT_PIN, HIGH);
    delay(20);
    sensor3.setTimeout(200);
    sensor3.init();
    sensor3.configureDefault();
    sensor3.setScaling(RANGE_SCALING);
    sensor3.setAddress(SENSOR3_ADDRESS);
    sensor3_present = !sensor3.timeoutOccurred();

    digitalWrite(SENSOR2_SHUT_PIN, HIGH);
    delay(20);
    sensor2.setTimeout(200);
    sensor2.init();
    sensor2.configureDefault();
    sensor2.setScaling(RANGE_SCALING);
    sensor2.setAddress(SENSOR2_ADDRESS);
    sensor2_present = !sensor2.timeoutOccurred();

    digitalWrite(SENSOR1_SHUT_PIN, HIGH);
    delay(20);
    sensor1.setTimeout(200);
    sensor1.init();
    sensor1.configureDefault();
    sensor1.setScaling(RANGE_SCALING);
    sensor1_present = !sensor1.timeoutOccurred();

    // Continuous mode: the sensors range on their own schedule and we
    // collect whatever is ready. No blocking waits in the main loop.
    if (sensor1_present) sensor1.startRangeContinuous(RANGE_PERIOD_MS);
    if (sensor2_present) sensor2.startRangeContinuous(RANGE_PERIOD_MS);
    if (sensor3_present) sensor3.startRangeContinuous(RANGE_PERIOD_MS);

#if VERBOSE_STARTUP
    Serial.print(F("# VL6180X s1="));
    Serial.print(sensor1_present ? F("ok") : F("MISSING"));
    Serial.print(F(" s2="));
    Serial.print(sensor2_present ? F("ok") : F("MISSING"));
    Serial.print(F(" s3="));
    Serial.println(sensor3_present ? F("ok") : F("MISSING"));
    if (!sensor1_present || !sensor2_present || !sensor3_present) {
        Serial.println(F("# Check SHUT pins 4/5/6, power, and the bus."));
    }
#endif
}

/* ---------------- Range-aided position correction ------------------- */

#if USE_RANGE_CORRECTION

// Correct one axis given the beam's component along it. This is the
// standard Kalman update for a measurement of a PROJECTION of position,
// H = [ux 0 uy 0 uz 0], split across three independent per-axis filters.
// A single joint 6-state filter would also build cross-axis covariance;
// we ignore that, which is the usual pragmatic simplification and costs
// little when the beam is close to one axis.
void correctAxisAlongBeam(PosVelKF *kf, float u, float innov, float S)
{
    if (u == 0.0f || S <= 0.0f) return;

    float K0 = kf->P[0][0] * u / S;
    float K1 = kf->P[1][0] * u / S;

    kf->pos += K0 * innov;
    kf->vel += K1 * innov;      // correcting position corrects velocity too

    float HP0 = u * kf->P[0][0];
    float HP1 = u * kf->P[0][1];
    kf->P[0][0] -= K0 * HP0;
    kf->P[0][1] -= K0 * HP1;
    kf->P[1][0] -= K1 * HP0;
    kf->P[1][1] -= K1 * HP1;
}

// Anchor, gate and apply one channel along its own beam.
// `stationary` comes from the IMU. When true we trust the accelerometer's
// verdict (no motion) over the rangefinder, because a range change while
// physically still can only be the target moving - e.g. a hand.
void updateRangeChannel(RangeChannel *ch, float ux, float uy, float uz,
                        float d, bool stationary)
{
    // Horizontal sensors (X, Y) get their vertical leakage removed so a
    // tilt cannot bleed their correction into Z. The vertical sensor is
    // exempt - correcting Z is its whole job.
    if (ch->horizontal_only) {
        uz = 0.0f;
        float hmag = sqrt(ux*ux + uy*uy);
        if (hmag < 0.3f) { ch->state = 0; return; }  // too vertical to use
        ux /= hmag; uy /= hmag;
    } else {
        // Vertical channel: strip any horizontal leakage instead, so a
        // tilt cannot bleed the up-sensor into X or Y. Bail if the beam
        // has tilted too far from vertical to mean anything.
        ux = 0.0f; uy = 0.0f;
        float vmag = fabs(uz);
        if (vmag < 0.3f) { ch->state = 0; return; }  // too horizontal to use
        uz = (uz > 0.0f) ? 1.0f : -1.0f;
    }

    float p_along = kf_pos_x.pos * ux + kf_pos_y.pos * uy + kf_pos_z.pos * uz;

    // --- Jump guard: a hand entering the beam is a step, not motion. ---
    // Real motion changes the range smoothly between samples. A single
    // large jump means the target itself changed, so freeze this channel
    // and let the reading settle rather than believing the step.
    if (ch->prev_mm >= 0.0f && fabs(d - ch->prev_mm) > RANGE_JUMP_MAX) {
        ch->prev_mm = d;
        ch->state = 3;             // rejected: target jumped
        // A jump also invalidates the surface we anchored to - re-anchor.
        ch->anchored = false;
        ch->stable_since_ms = 0;
        return;
    }
    ch->prev_mm = d;

    if (!ch->anchored) {
        if (ch->last_mm < 0.0f || fabs(d - ch->last_mm) > RANGE_STABLE_MM) {
            ch->stable_since_ms = 0;
        } else if (ch->stable_since_ms == 0) {
            ch->stable_since_ms = millis();
        }
        ch->last_mm = d;

        if (ch->stable_since_ms == 0
            || millis() - ch->stable_since_ms < RANGE_ANCHOR_MS) {
            ch->state = 1;
            return;
        }

        ch->anchor_C = p_along + d;
        ch->anchored = true;
        ch->reject_count = 0;
    }
    ch->last_mm = d;

    ch->innov = (ch->anchor_C - d) - p_along;

    // --- Deciding whether to trust this reading -----------------------
    // The old rule "don't correct while stationary" was wrong: a SLOW
    // forward push produces almost no acceleration, so the IMU still
    // reads stationary, and re-baselining the anchor every sample made
    // that motion invisible - the sensor watched you approach the target
    // while the code kept calling the new distance "zero".
    //
    // The real tell for a hand vs. real motion is not the accelerometer,
    // it is whether the range is changing SMOOTHLY. A hand block is a
    // step (already caught by the jump guard above). A steady creep is
    // motion, and must be corrected even when the IMU calls it stationary.
    //
    // So: only re-baseline when the device is BOTH stationary AND the
    // range is genuinely flat (not creeping). Otherwise, correct.
#if RANGE_TRUST_WHILE_MOVING_ONLY
    float creep = fabs(d - ch->last_mm);   // change since previous accepted mm
    if (stationary && creep < RANGE_CREEP_MM) {
        ch->anchor_C = p_along + d;   // truly still: adopt reading as zero
        ch->innov = 0.0f;
        ch->state = 2;
        ch->reject_count = 0;
        ch->last_mm = d;
        return;
    }
#endif

    if (fabs(ch->innov) > RANGE_INNOV_MAX) {
        ch->state = 3;
        if (++ch->reject_count > RANGE_REJECT_MAX) {
            ch->anchor_C = p_along + d;      // stale reference; adopt current
            ch->reject_count = 0;
        }
        return;
    }
    ch->reject_count = 0;

    float S = ux * ux * kf_pos_x.P[0][0]
            + uy * uy * kf_pos_y.P[0][0]
            + uz * uz * kf_pos_z.P[0][0]
            + RANGE_MEAS_VAR;

    correctAxisAlongBeam(&kf_pos_x, ux, ch->innov, S);
    correctAxisAlongBeam(&kf_pos_y, uy, ch->innov, S);
    correctAxisAlongBeam(&kf_pos_z, uz, ch->innov, S);
    ch->state = 2;
}

void resetChannel(RangeChannel *ch)
{
    ch->state = 0;
    ch->anchored = false;
    ch->stable_since_ms = 0;
    ch->last_mm = -1.0f;
    ch->prev_mm = -1.0f;
}

void applyRangeCorrection(bool is_stationary)
{
    // Rotate each beam from the body frame into the world frame using the
    // current fused orientation. This keeps every correction landing on
    // the right world axis even as the syringe tilts and turns.
    float u1x, u1y, u1z, u2x, u2y, u2z, u3x, u3y, u3z;
    rotateBodyToWorld(ch1.bx, ch1.by, ch1.bz,
                      my_95Q.roll / 100.0f, my_95Q.pitch / 100.0f,
                      referencedYaw(), &u1x, &u1y, &u1z);
    rotateBodyToWorld(ch2.bx, ch2.by, ch2.bz,
                      my_95Q.roll / 100.0f, my_95Q.pitch / 100.0f,
                      referencedYaw(), &u2x, &u2y, &u2z);
    rotateBodyToWorld(ch3.bx, ch3.by, ch3.bz,
                      my_95Q.roll / 100.0f, my_95Q.pitch / 100.0f,
                      referencedYaw(), &u3x, &u3y, &u3z);

    // Three perpendicular sensors, one per axis. Each corrects its own
    // channel independently; a channel with no valid reading resets so it
    // re-anchors cleanly when its target reappears. (The old two-sensor
    // "parallel beams -> average" special case is gone: with X, Y and Z
    // beams mutually perpendicular it can never trigger.)
    if (dist1_mm >= 0.0f) updateRangeChannel(&ch1, u1x, u1y, u1z, dist1_mm, is_stationary);
    else                  resetChannel(&ch1);

    if (dist2_mm >= 0.0f) updateRangeChannel(&ch2, u2x, u2y, u2z, dist2_mm, is_stationary);
    else                  resetChannel(&ch2);

    if (dist3_mm >= 0.0f) updateRangeChannel(&ch3, u3x, u3y, u3z, dist3_mm, is_stationary);
    else                  resetChannel(&ch3);
}
#endif  // USE_RANGE_CORRECTION

/* ---------------- Sample acquisition -------------------------------- */

bool sampleReady()
{
    if (ready_Ok) {
        ready_Ok = 0;
        return true;
    }
#if USE_POLL_FALLBACK
    if (millis() - last_sample_ms >= (unsigned long)(delay_t)) return true;
#endif
    return false;
}

bool readIMU()
{
    unsigned char data[27] = {0};
    if (!iic_read(0x08, data, 27)) {
        i2c_fail_count++;
        return false;
    }
    memcpy(&my_95Q, data, 27);
    last_sample_ms = millis();
    sample_count++;
    return true;
}

// Single-character commands, typed into the Serial Monitor:
//   a = drop the anchor and re-establish it here
//   z = zero position and velocity
void serviceSerialCommands()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'a' || c == 'A') {
            resetChannel(&ch1);
            resetChannel(&ch2);
            resetChannel(&ch3);
#if VERBOSE_STARTUP
            Serial.println(F("# Re-anchoring range reference..."));
#endif
        } else if (c == 'z' || c == 'Z') {
            posvelInit(&kf_pos_x, 0.0f, 0.0f, 1.0f, 1.0f);
            posvelInit(&kf_pos_y, 0.0f, 0.0f, 1.0f, 1.0f);
            posvelInit(&kf_pos_z, 0.0f, 0.0f, 1.0f, 1.0f);
            resetChannel(&ch1);
            resetChannel(&ch2);
            resetChannel(&ch3);
#if VERBOSE_STARTUP
            Serial.println(F("# Position zeroed."));
#endif
        }
    }
}

void serviceDiagnostics()
{
#if VERBOSE_STARTUP
    unsigned long now = millis();
    if (now - last_diag_ms < DIAG_PERIOD_MS) return;
    last_diag_ms = now;

    if (sample_count > 0) { sample_count = 0; return; }

    Serial.print(F("# NO DATA. int_fires="));
    Serial.print(int_count);
    Serial.print(F(" i2c_fails="));
    Serial.print(i2c_fail_count);
    if (int_count == 0) Serial.print(F("  -> data-ready pin is not reaching D2"));
    else if (i2c_fail_count > 0) Serial.print(F("  -> I2C reads failing: check SDA/SCL, pull-ups, address"));
    Serial.println();
    i2c_fail_count = 0;
#endif
}

/* ---------------- Startup calibration ------------------------------- */

void calibrate()
{
#if VERBOSE_STARTUP
    Serial.println(F("# Calibrating - hold the device COMPLETELY still..."));
#endif

    int   collected = 0;
    float sum_wx = 0, sum_wy = 0, sum_wz = 0;
    float sum_am = 0, sum_am2 = 0;
    float sum_gm = 0, sum_gm2 = 0;
    unsigned long cal_start = millis();

    while (collected < CAL_SAMPLES && millis() - cal_start < 8000UL) {
        if (!sampleReady()) continue;
        if (!readIMU())     continue;

        if (collected == 0) yaw_ref_deg = my_95Q.yaw / 100.0f;

        float ax_g = my_95Q.acc_x * ACC_SCALE;
        float ay_g = my_95Q.acc_y * ACC_SCALE;
        float az_g = my_95Q.acc_z * ACC_SCALE;

        float wx, wy, wz;
        rotateBodyToWorld(ax_g, ay_g, az_g,
                          my_95Q.roll / 100.0f, my_95Q.pitch / 100.0f,
                          referencedYaw(), &wx, &wy, &wz);

        sum_wx += wx; sum_wy += wy; sum_wz += wz;

        float am = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        sum_am += am; sum_am2 += am * am;

        float gx  = my_95Q.gyro_x * GYRO_SCALE;
        float gy_ = my_95Q.gyro_y * GYRO_SCALE;
        float gz  = my_95Q.gyro_z * GYRO_SCALE;
        float gm  = sqrt(gx*gx + gy_*gy_ + gz*gz);
        sum_gm += gm; sum_gm2 += gm * gm;

        collected++;
    }

    sample_count = 0;

    if (collected < 20) {
#if VERBOSE_STARTUP
        Serial.print(F("# CALIBRATION FAILED. samples=")); Serial.print(collected);
        Serial.print(F(" int_fires=")); Serial.print(int_count);
        Serial.print(F(" i2c_fails=")); Serial.println(i2c_fail_count);
        Serial.println(F("# Running anyway with zero bias."));
#endif
        return;
    }

    float n = (float)collected;

    float mean_am = sum_am / n;
    float std_am  = sqrt(fmax(0.0f, sum_am2 / n - mean_am * mean_am));
    float mean_gm = sum_gm / n;
    float std_gm  = sqrt(fmax(0.0f, sum_gm2 / n - mean_gm * mean_gm));

    // Scale correction first: at rest |a| must be exactly 1g, so the
    // factor that makes it so is the true scale error. This has to come
    // before the bias, because bias is what is left AFTER scaling.
    if (mean_am > 0.5f && mean_am < 1.5f) {
        acc_gain = 1.0f / mean_am;
        if (acc_gain < ACC_GAIN_MIN) acc_gain = ACC_GAIN_MIN;
        if (acc_gain > ACC_GAIN_MAX) acc_gain = ACC_GAIN_MAX;
    }

    // Sums were accumulated ungained; rotation is linear, so scaling the
    // averages afterwards is equivalent to having scaled every sample.
    acc_bias_world_x = (sum_wx / n) * acc_gain * ACC_G_TO_MMS2;
    acc_bias_world_y = (sum_wy / n) * acc_gain * ACC_G_TO_MMS2;
    acc_bias_world_z = ((sum_wz / n) * acc_gain - GRAVITY_SIGN * 1.0f) * ACC_G_TO_MMS2;

    std_am *= acc_gain;
    tol_acc_std    = fmax(3.0f * std_am, 0.003f);
    tol_acc_offset = fmax(6.0f * std_am, 0.012f);
    tol_gyro       = fmax(mean_gm + 6.0f * std_gm, 3.0f);

#if VERBOSE_STARTUP
    Serial.print(F("# OK samples=")); Serial.println(collected);
    Serial.print(F("# Accel scale gain=")); Serial.print(acc_gain, 5);
    Serial.print(F("  (raw |a| was ")); Serial.print(mean_am, 5);
    Serial.println(F(" g at rest; 1.00000 would be perfect)"));
    Serial.print(F("# Bias mm/s^2 x=")); Serial.print(acc_bias_world_x, 2);
    Serial.print(F(" y=")); Serial.print(acc_bias_world_y, 2);
    Serial.print(F(" z=")); Serial.println(acc_bias_world_z, 2);
    Serial.print(F("# ZUPT acc_off<")); Serial.print(tol_acc_offset, 4);
    Serial.print(F(" acc_std<")); Serial.print(tol_acc_std, 4);
    Serial.print(F(" gyro<")); Serial.println(tol_gyro, 3);
#endif
}

/* ---------------- setup / loop -------------------------------------- */

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(400);

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    delay(100);

    byte td = 1;
    bool td_ok = iic_read(0x02, &td, 1);
    if (td_ok) {
        switch (td) {
            case 0: delay_t = 100; break;
            case 1: delay_t = 20;  break;
            case 2: delay_t = 10;  break;
            case 3: delay_t = 5;   break;
            default: delay_t = 20; break;
        }
    } else {
        delay_t = 20;
#if VERBOSE_STARTUP
        Serial.println(F("# WARNING: first IMU read failed. Check SDA/SCL/power/address."));
#endif
    }
    delay(delay_t);

    attachInterrupt(digitalPinToInterrupt(2), Exti, RISING);

    setupRangeSensors();

    kalmanInit(&kf_acc_x,  0.02f, 0.05f, 10.0f);
    kalmanInit(&kf_acc_y,  0.02f, 0.05f, 10.0f);
    kalmanInit(&kf_acc_z,  0.02f, 0.05f, 10.0f);
    kalmanInit(&kf_gyro_x, 0.05f, 0.5f,  10.0f);
    kalmanInit(&kf_gyro_y, 0.05f, 0.5f,  10.0f);
    kalmanInit(&kf_gyro_z, 0.05f, 0.5f,  10.0f);
    kalmanInit(&kf_dist,   0.5f,  9.0f,  1000.0f);
    kalmanInit(&kf_dist2,  0.5f,  9.0f,  1000.0f);
    kalmanInit(&kf_dist3,  0.5f,  9.0f,  1000.0f);

    posvelInit(&kf_pos_x, 0.0f, 0.0f, 1.0f, 1.0f);
    posvelInit(&kf_pos_y, 0.0f, 0.0f, 1.0f, 1.0f);
    posvelInit(&kf_pos_z, 0.0f, 0.0f, 1.0f, 1.0f);

    last_sample_ms = millis();
    calibrate();

    last_update_us = micros();
    last_print_ms  = millis();
    last_diag_ms   = millis();
}

void loop()
{
    serviceSerialCommands();
    serviceDiagnostics();
    serviceRangeSensors();       // non-blocking; takes whatever is ready

    if (!sampleReady()) return;
    if (!readIMU())     return;

    unsigned long now_us = micros();
    float dt = (now_us - last_update_us) / 1000000.0f;
    last_update_us = now_us;
    if (dt <= 0.0f || dt > 0.25f) dt = delay_t / 1000.0f;

    // acc_gain removes the datasheet-vs-reality scale error. Without it a
    // half-percent error becomes ~50 mm/s^2 of phantom vertical
    // acceleration the moment you tilt away from the calibration pose.
    float acc_x_g = my_95Q.acc_x * ACC_SCALE * acc_gain;
    float acc_y_g = my_95Q.acc_y * ACC_SCALE * acc_gain;
    float acc_z_g = my_95Q.acc_z * ACC_SCALE * acc_gain;

    float gyro_x_dps = my_95Q.gyro_x * GYRO_SCALE;
    float gyro_y_dps = my_95Q.gyro_y * GYRO_SCALE;
    float gyro_z_dps = my_95Q.gyro_z * GYRO_SCALE;

    float acc_x_mms2 = kalmanUpdate(&kf_acc_x, acc_x_g) * ACC_G_TO_MMS2;
    float acc_y_mms2 = kalmanUpdate(&kf_acc_y, acc_y_g) * ACC_G_TO_MMS2;
    float acc_z_mms2 = kalmanUpdate(&kf_acc_z, acc_z_g) * ACC_G_TO_MMS2;

    float gyro_x_filt = kalmanUpdate(&kf_gyro_x, gyro_x_dps);
    float gyro_y_filt = kalmanUpdate(&kf_gyro_y, gyro_y_dps);
    float gyro_z_filt = kalmanUpdate(&kf_gyro_z, gyro_z_dps);

    // Navigation channel uses raw accel: the posvel filter has its own
    // noise model, so smoothing first would double-count the filtering.
    float wx_g, wy_g, wz_g;
    rotateBodyToWorld(acc_x_g, acc_y_g, acc_z_g,
                      my_95Q.roll / 100.0f, my_95Q.pitch / 100.0f,
                      referencedYaw(), &wx_g, &wy_g, &wz_g);

    float res_x = wx_g * ACC_G_TO_MMS2;
    float res_y = wy_g * ACC_G_TO_MMS2;
    float res_z = (wz_g - GRAVITY_SIGN * 1.0f) * ACC_G_TO_MMS2;

    float world_ax = res_x - acc_bias_world_x;
    float world_ay = res_y - acc_bias_world_y;
    float world_az = res_z - acc_bias_world_z;

    float acc_mag_g  = sqrt(acc_x_g*acc_x_g + acc_y_g*acc_y_g + acc_z_g*acc_z_g);
    float gyro_mag_d = sqrt(gyro_x_dps*gyro_x_dps + gyro_y_dps*gyro_y_dps + gyro_z_dps*gyro_z_dps);
    statPush(acc_mag_g, gyro_mag_d);

    bool stat_raw = statIsStationary();
    if (!stat_raw) stationary_since_ms = 0;
    else if (stationary_since_ms == 0) stationary_since_ms = millis();
    bool is_stationary = stat_raw && (millis() - stationary_since_ms >= STAT_HOLD_MS);

    posvelPredict(&kf_pos_x, world_ax, dt, Q_ACCEL_VAR);
    posvelPredict(&kf_pos_y, world_ay, dt, Q_ACCEL_VAR);
    posvelPredict(&kf_pos_z, world_az, dt, Q_ACCEL_VAR);

#if USE_RANGE_CORRECTION
    applyRangeCorrection(is_stationary);
#endif

    if (is_stationary) {
        posvelZUPT(&kf_pos_x, 1.0f);
        posvelZUPT(&kf_pos_y, 1.0f);
        posvelZUPT(&kf_pos_z, 1.0f);

        acc_bias_world_x += BIAS_TRACK_A * (res_x - acc_bias_world_x);
        acc_bias_world_y += BIAS_TRACK_A * (res_y - acc_bias_world_y);
        acc_bias_world_z += BIAS_TRACK_A * (res_z - acc_bias_world_z);
    } else if (VEL_TAU_S > 0.0f) {
        float leak = exp(-dt / VEL_TAU_S);
        posvelLeak(&kf_pos_x, leak);
        posvelLeak(&kf_pos_y, leak);
        posvelLeak(&kf_pos_z, leak);
    }

    unsigned long now_ms = millis();
    if (now_ms - last_print_ms < (unsigned long)(1000 / PRINT_HZ)) return;
    last_print_ms = now_ms;

    // --- Core: sent every line, this is what moves the model ---
    Serial.print(F("ro:"));   Serial.print(my_95Q.roll / 100.0, 1);
    Serial.print(F(",pi:"));  Serial.print(my_95Q.pitch / 100.0, 1);
    Serial.print(F(",ya:"));  Serial.print(my_95Q.yaw / 100.0, 1);
    Serial.print(F(",px:"));  Serial.print(kf_pos_x.pos, 1);
    Serial.print(F(",py:"));  Serial.print(kf_pos_y.pos, 1);
    Serial.print(F(",pz:"));  Serial.print(kf_pos_z.pos, 1);
    Serial.print(F(",st:"));  Serial.print(is_stationary ? 1 : 0);

    // --- Extras: one group per line, cycling. Each refreshes at
    //     PRINT_HZ / 5, which is plenty for a numeric readout. ---
    switch (extras_group) {
    case 0:
        Serial.print(F(",ax:")); Serial.print(acc_x_mms2, 0);
        Serial.print(F(",ay:")); Serial.print(acc_y_mms2, 0);
        Serial.print(F(",az:")); Serial.print(acc_z_mms2, 0);
        break;
    case 1:
        Serial.print(F(",gx:")); Serial.print(gyro_x_filt, 2);
        Serial.print(F(",gy:")); Serial.print(gyro_y_filt, 2);
        Serial.print(F(",gz:")); Serial.print(gyro_z_filt, 2);
        break;
    case 2:
        Serial.print(F(",mx:")); Serial.print(my_95Q.mag_x);
        Serial.print(F(",my:")); Serial.print(my_95Q.mag_y);
        Serial.print(F(",mz:")); Serial.print(my_95Q.mag_z);
        break;
    case 3:
        // Channel 1 (X / right): distance, state, innovation.
        Serial.print(F(",d1:"));  Serial.print(dist1_mm, 0);
        Serial.print(F(",r1s:")); Serial.print(ch1.state);
        Serial.print(F(",r1i:")); Serial.print(ch1.innov, 1);
        break;
    case 4:
        // Channel 2 (Y / forward): distance, state, innovation.
        Serial.print(F(",d2:"));  Serial.print(dist2_mm, 0);
        Serial.print(F(",r2s:")); Serial.print(ch2.state);
        Serial.print(F(",r2i:")); Serial.print(ch2.innov, 1);
        break;
    default:
        // Channel 3 (Z / up): distance, state, innovation, plus temp.
        Serial.print(F(",d3:"));  Serial.print(dist3_mm, 0);
        Serial.print(F(",r3s:")); Serial.print(ch3.state);
        Serial.print(F(",r3i:")); Serial.print(ch3.innov, 1);
        Serial.print(F(",tp:"));  Serial.print(my_95Q.temp / 100.0, 1);
        break;
    }
    if (++extras_group > 5) extras_group = 0;

    Serial.println();
}

/* ---------------- low level ----------------------------------------- */

void Exti()
{
    ready_Ok = 1;
    int_count++;
}

bool iic_read(unsigned char add, unsigned char *data, unsigned char len)
{
    Wire.beginTransmission(iic_add);
    Wire.write(add);
    if (Wire.endTransmission(false) != 0) return false;

    byte received = Wire.requestFrom(iic_add, (int)len);
    if (received != len) return false;

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
