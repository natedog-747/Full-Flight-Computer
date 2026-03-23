#pragma once
#include <math.h>
#include <stdint.h>

// ── Error-State Kalman Filter (ESKF) ─────────────────────────────────────────
//
// Nominal state (22 scalars):
//   pos[3]        – NED position   (m)
//   vel[3]        – NED velocity   (m/s)
//   q[4]          – body-to-NED quaternion [w, x, y, z]
//   biasAccel[3]  – IMU accel bias (m/s²)
//   biasGyro[3]   – IMU gyro bias  (rad/s)
//   baroBias      – baro bias      (m)
//   gpsBiasPos[3] – GPS pos bias   (m, NED)      ← anti-drift states
//   gpsBiasVel[2] – GPS vel bias   (m/s, NE only; velD not in NMEA)
//
// Error state δx (21-D):
//   δp    [0-2]  – position error        (m)
//   δv    [3-5]  – velocity error        (m/s)
//   δθ    [6-8]  – attitude error        (rad, small-angle, NED local frame)
//   δab   [9-11] – IMU accel bias error  (m/s²)
//   δwb  [12-14] – IMU gyro bias error   (rad/s)
//   δbb   [15]   – baro bias error       (m)
//   δbgp [16-18] – GPS pos bias error    (m)
//   δbgv [19-20] – GPS vel bias NE error (m/s)
//
// GPS measurement model:
//   z_pos = p_true + gpsBiasPos + noise       H[δp]=I, H[δbgp]=I
//   z_vel = v_true_NE + gpsBiasVel_NE + noise H[δv_NE]=I, H[δbgv]=I
// The GPS bias states evolve as random walks, letting the filter track and
// remove the slowly-varying GPS position/velocity offset.
//
// Propagation: Fx ≈ I + Fc·dt  (first-order discrete, Sola §5.4)
// Noise:       P += (Fc·P + P·Fc^T)·dt + Qd
//
// Reference: Sola, "A micro Lie theory" (arXiv:1711.02508)
// ─────────────────────────────────────────────────────────────────────────────

class KalmanFilter {
public:
    // ── Tuning parameters ─────────────────────────────────────────────────────
    static constexpr float INIT_ACCEL_SECS  = 10.0f;  // accel-avg duration (s)
    static constexpr float GPS_YAW_SPEED_MS =  1.0f;  // min GPS speed to accept heading (m/s)

    // Process noise — standard deviations (SI units)
    static constexpr float SIGMA_ACCEL        =  0.30f;  // m/s²      IMU accel white noise
    static constexpr float SIGMA_GYRO         =  0.01f;  // rad/s     IMU gyro white noise
    static constexpr float SIGMA_ACCEL_WALK   =  3e-3f;  // m/s²/√s  accel bias random walk
    static constexpr float SIGMA_GYRO_WALK    =  1e-5f;  // rad/s/√s gyro bias random walk
    static constexpr float SIGMA_BARO_WALK    =  0.01f;  // m/√s     baro bias random walk
    static constexpr float SIGMA_GPS_POS_WALK =  0.10f;  // m/√s     GPS pos bias random walk
    static constexpr float SIGMA_GPS_VEL_WALK =  0.01f;  // m/s/√s   GPS vel bias random walk

    // Measurement noise — standard deviations (SI)
    static constexpr float SIGMA_BARO_MEAS  =  0.50f;  // m    barometer altitude noise
    static constexpr float SIGMA_GPS_POS    =  3.00f;  // m    GPS pos noise @ HDOP=1
    static constexpr float SIGMA_GPS_VEL    =  0.30f;  // m/s  GPS vel noise @ HDOP=1
    static constexpr float SIGMA_GPS_YAW    =  5.00f;  // deg  GPS heading noise

    // Error-state dimension — use literal 21 in array declarations to avoid
    // collisions with toolchain macros (N, DIM, etc.).
    static constexpr int ESKF_STATE_DIM = 21;

    // ── Initialisation phase ──────────────────────────────────────────────────
    enum class InitPhase : uint8_t {
        ACCEL_AVG = 0,  // averaging accel/gyro → roll/pitch + gyro bias
        AWAIT_YAW = 1,  // roll/pitch locked; waiting for GPS heading ≥ speed thresh
        READY     = 2   // fully initialised; ESKF running
    };

    KalmanFilter() { reset(); }
    void reset();

    // ── Initialisation feeds (call every IMU cycle) ───────────────────────────
    void feedImu(float gxNed, float gyNed, float gzNed,
                 float axNed, float ayNed, float azNed);
    void feedGpsHeading(float speedMs, float headingDeg);

    // ── Propagation (call every IMU cycle after init feeds) ──────────────────
    void predict(float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float dt);

    // ── Measurement updates (call ONLY when new sensor data arrives) ─────────

    // GPS position (m NED from locked origin), NED velocity (m/s, velD ignored),
    // HDOP, heading (deg 0=North CW), speed (m/s).
    void updateFromGPS(float posN, float posE, float posD,
                       float velN, float velE,
                       float hdop, float headingDeg, float speedMs);

    // Barometer: altitude above ground, positive-up (m).
    void updateFromBarometer(float altM);

    // ── Getters ───────────────────────────────────────────────────────────────
    InitPhase getInitPhase() const { return mInitPhase; }
    bool      isReady()      const { return mInitPhase == InitPhase::READY; }

    void getQuaternion(float &w, float &x, float &y, float &z) const;
    void getEulerDeg(float &roll, float &pitch, float &yaw) const;
    void getPosition(float &posN, float &posE, float &posD) const;
    void getVelocity(float &velN, float &velE, float &velD) const;
    void getGyroBias(float &bgx, float &bgy, float &bgz) const;
    float getBaroBias() const { return mBaroBias; }
    void getGpsBiasPos(float &bN, float &bE, float &bD) const;
    void getGpsBiasVel(float &bN, float &bE) const;

private:
    // ── Nominal state ─────────────────────────────────────────────────────────
    float mPos[3]          = {0,0,0};
    float mVel[3]          = {0,0,0};
    float mQuat[4]         = {1,0,0,0};  // [w, x, y, z]
    float mBiasAccel[3]    = {0,0,0};    // m/s²
    float mBiasGyro[3]     = {0,0,0};    // rad/s
    float mBaroBias        = 0.0f;       // m
    float mGpsBiasPos[3]   = {0,0,0};   // m  (NED GPS position bias)
    float mGpsBiasVel[2]   = {0,0};     // m/s (NE GPS velocity bias)

    // ── Error-state covariance (21×21, row-major) ─────────────────────────────
    float mCov[21][21];

    // ── Initialisation accumulators ───────────────────────────────────────────
    InitPhase mInitPhase   = InitPhase::ACCEL_AVG;
    double    mAccumGx = 0, mAccumGy = 0, mAccumGz = 0;
    double    mAccumAx = 0, mAccumAy = 0, mAccumAz = 0;
    uint32_t  mAccumCount  = 0;
    float     mInitRoll    = 0.0f;
    float     mInitPitch   = 0.0f;

    // ── Private helpers ───────────────────────────────────────────────────────
    void normalizeQuat();
    void setQuatFromEuler(float roll, float pitch, float yaw);
    void buildDCM(float R[3][3]) const;
    void injectErrorState(const float dx[21]);
    void symmetrizeCov();
    static bool mat5Invert(float A[5][5], float Ainv[5][5]);
};
