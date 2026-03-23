#pragma once
#include <math.h>
#include <stdint.h>

// ── Error-State Kalman Filter (ESKF) ─────────────────────────────────────────
//
// Nominal state (17 scalars):
//   pos[3]       – NED position   (m)
//   vel[3]       – NED velocity   (m/s)
//   q[4]         – body-to-NED quaternion [w, x, y, z]
//   biasAccel[3] – accel bias     (m/s²)
//   biasGyro[3]  – gyro bias      (rad/s)
//   baroBias     – baro bias      (m)
//
// Error state δx (16-D):
//   δp  [0-2]  – position error   (m)
//   δv  [3-5]  – velocity error   (m/s)
//   δθ  [6-8]  – attitude error   (rad, small-angle, NED local frame)
//   δab [9-11] – accel bias error (m/s²)
//   δwb [12-14]– gyro bias error  (rad/s)
//   δb  [15]   – baro bias error  (m)
//
// Propagation: Fx ≈ I + Fc·dt  (first-order discrete, Sola §5.4)
// Noise:       P += (Fc·P + P·Fc^T)·dt + Qd  then diagonal Qd added
//
// Reference: Sola, "A micro Lie theory" (arXiv:1711.02508)
// ─────────────────────────────────────────────────────────────────────────────

class KalmanFilter {
public:
    // ── Tuning parameters ─────────────────────────────────────────────────────
    static constexpr float INIT_ACCEL_SECS  = 10.0f;  // accel-avg duration (s)
    static constexpr float GPS_YAW_SPEED_MS =  1.0f;  // min GPS speed to accept heading (m/s)

    // Process noise — standard deviations (SI units / sqrt(Hz) spectral densities)
    static constexpr float SIGMA_ACCEL      =  0.30f;  // m/s²      IMU accel white noise
    static constexpr float SIGMA_GYRO       =  0.01f;  // rad/s     IMU gyro white noise
    static constexpr float SIGMA_ACCEL_WALK =  3e-3f;  // m/s²/√s  accel bias random walk
    static constexpr float SIGMA_GYRO_WALK  =  1e-5f;  // rad/s/√s gyro bias random walk
    static constexpr float SIGMA_BARO_WALK  =  0.01f;  // m/√s     baro bias random walk

    // Measurement noise — standard deviations (SI)
    static constexpr float SIGMA_BARO_MEAS  =  0.50f;  // m    barometer altitude noise
    static constexpr float SIGMA_GPS_POS    =  3.00f;  // m    GPS position noise @ HDOP=1
    static constexpr float SIGMA_GPS_VEL    =  0.30f;  // m/s  GPS velocity noise @ HDOP=1
    static constexpr float SIGMA_GPS_YAW    =  5.00f;  // deg  GPS heading noise

    // Error-state dimension (keep as constexpr, use literal 16 for array sizes
    // to avoid collisions with toolchain macros like N, DIM, etc.)
    static constexpr int ESKF_STATE_DIM = 16;

    // ── Initialisation phase ──────────────────────────────────────────────────
    enum class InitPhase : uint8_t {
        ACCEL_AVG = 0,  // averaging accel/gyro → roll/pitch + gyro bias
        AWAIT_YAW = 1,  // roll/pitch locked; waiting for GPS heading ≥ speed thresh
        READY     = 2   // fully initialised; ESKF running
    };

    KalmanFilter() { reset(); }

    // Full state reset — re-enters ACCEL_AVG
    void reset();

    // ── Initialisation feeds (call every IMU cycle) ───────────────────────────
    // NED-frame gyro (rad/s) and accel (m/s²) — sign-flipped from BNO055 body frame
    void feedImu(float gxNed, float gyNed, float gzNed,
                 float axNed, float ayNed, float azNed);

    // GPS speed (m/s) and course-over-ground (deg, 0=North CW)
    // Transitions AWAIT_YAW → READY when speed ≥ GPS_YAW_SPEED_MS
    void feedGpsHeading(float speedMs, float headingDeg);

    // ── Propagation (call every IMU cycle after init feeds) ──────────────────
    // NED-frame gyro (rad/s) and accel (m/s²); dt in seconds
    void predict(float gx, float gy, float gz,
                 float ax, float ay, float az,
                 float dt);

    // ── Measurement updates (call ONLY when new sensor data arrives) ─────────

    // GPS position (m, NED from locked origin), NED velocity (m/s — velD unused),
    // HDOP, heading (deg, 0=North CW), speed (m/s).
    // velD is intentionally absent — not available in standard NMEA.
    void updateFromGPS(float posN, float posE, float posD,
                       float velN, float velE,
                       float hdop, float headingDeg, float speedMs);

    // Barometer: altitude above ground, positive-up (m)
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

private:
    // ── Nominal state ─────────────────────────────────────────────────────────
    float mPos[3]       = {0,0,0};
    float mVel[3]       = {0,0,0};
    float mQuat[4]      = {1,0,0,0};  // [w, x, y, z]
    float mBiasAccel[3] = {0,0,0};    // m/s²
    float mBiasGyro[3]  = {0,0,0};    // rad/s
    float mBaroBias     = 0.0f;       // m

    // ── Error-state covariance (16×16, row-major) ─────────────────────────────
    // Literal 16 used here to avoid collisions with toolchain macros.
    float mCov[16][16];

    // ── Initialisation accumulators ───────────────────────────────────────────
    InitPhase mInitPhase   = InitPhase::ACCEL_AVG;
    double    mAccumGx     = 0;
    double    mAccumGy     = 0;
    double    mAccumGz     = 0;
    double    mAccumAx     = 0;
    double    mAccumAy     = 0;
    double    mAccumAz     = 0;
    uint32_t  mAccumCount  = 0;
    float     mInitRoll    = 0.0f;  // rad — preserved so yaw can be set later
    float     mInitPitch   = 0.0f;  // rad

    // ── Private helpers ───────────────────────────────────────────────────────
    void normalizeQuat();
    void setQuatFromEuler(float roll, float pitch, float yaw);  // angles in rad
    void buildDCM(float R[3][3]) const;  // body-to-NED rotation matrix
    void injectErrorState(const float dx[16]);
    void symmetrizeCov();

    // Invert a 5×5 matrix via Gaussian elimination with partial pivoting.
    // Returns false if matrix is singular.
    static bool mat5Invert(float A[5][5], float Ainv[5][5]);
};
