#include "KalmanFilter.h"
#include <string.h>  // memset

// ── reset ─────────────────────────────────────────────────────────────────────
void KalmanFilter::reset() {
    mPos[0] = mPos[1] = mPos[2] = 0.0f;
    mVel[0] = mVel[1] = mVel[2] = 0.0f;
    mQuat[0] = 1.0f; mQuat[1] = mQuat[2] = mQuat[3] = 0.0f;
    mBiasAccel[0] = mBiasAccel[1] = mBiasAccel[2] = 0.0f;
    mBiasGyro[0]  = mBiasGyro[1]  = mBiasGyro[2]  = 0.0f;
    mBaroBias = 0.0f;

    memset(mCov, 0, sizeof(mCov));

    // Initial diagonal covariance — indices [δp δv δθ δab δwb δbaro]
    const float diag[16] = {
        0.0f,   0.0f,   0.0f,    // δp  — start position known (NED origin)
        0.0f,   0.0f,   0.0f,    // δv  — start velocity known (stationary)
        0.1f,   0.1f,   1.0f,    // δθ  — roll/pitch small, yaw unknown
        1e-4f,  1e-4f,  1e-4f,   // δab — small initial accel bias uncertainty
        1e-6f,  1e-6f,  1e-6f,   // δwb — small initial gyro bias uncertainty
        1.0f                     // δbb — baro bias unknown (1 m std)
    };
    for (int i = 0; i < 16; i++) mCov[i][i] = diag[i];

    mInitPhase     = InitPhase::ACCEL_AVG;
    mAccumGx       = 0; mAccumGy = 0; mAccumGz = 0;
    mAccumAx       = 0; mAccumAy = 0; mAccumAz = 0;
    mAccumCount    = 0;
    mAwaitYawCount = 0;
    mInitRoll      = 0.0f;
    mInitPitch     = 0.0f;
}

// ── normalizeQuat ─────────────────────────────────────────────────────────────
void KalmanFilter::normalizeQuat() {
    float len = sqrtf(mQuat[0]*mQuat[0] + mQuat[1]*mQuat[1] +
                      mQuat[2]*mQuat[2] + mQuat[3]*mQuat[3]);
    if (len < 1e-9f) { mQuat[0] = 1.0f; mQuat[1] = mQuat[2] = mQuat[3] = 0.0f; return; }
    mQuat[0] /= len; mQuat[1] /= len; mQuat[2] /= len; mQuat[3] /= len;
}

// ── setQuatFromEuler ──────────────────────────────────────────────────────────
// ZYX convention (yaw first, then pitch, then roll); angles in radians.
void KalmanFilter::setQuatFromEuler(float roll, float pitch, float yaw) {
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    mQuat[0] = cr*cp*cy + sr*sp*sy;
    mQuat[1] = sr*cp*cy - cr*sp*sy;
    mQuat[2] = cr*sp*cy + sr*cp*sy;
    mQuat[3] = cr*cp*sy - sr*sp*cy;
}

// ── buildDCM ──────────────────────────────────────────────────────────────────
// Body-to-NED rotation matrix from current quaternion (Sola eq.115).
void KalmanFilter::buildDCM(float R[3][3]) const {
    float qw = mQuat[0], qx = mQuat[1], qy = mQuat[2], qz = mQuat[3];
    R[0][0] = qw*qw + qx*qx - qy*qy - qz*qz;
    R[0][1] = 2.0f*(qx*qy - qw*qz);
    R[0][2] = 2.0f*(qx*qz + qw*qy);
    R[1][0] = 2.0f*(qx*qy + qw*qz);
    R[1][1] = qw*qw - qx*qx + qy*qy - qz*qz;
    R[1][2] = 2.0f*(qy*qz - qw*qx);
    R[2][0] = 2.0f*(qx*qz - qw*qy);
    R[2][1] = 2.0f*(qy*qz + qw*qx);
    R[2][2] = qw*qw - qx*qx - qy*qy + qz*qz;
}

// ── symmetrizeCov ─────────────────────────────────────────────────────────────
void KalmanFilter::symmetrizeCov() {
    for (int i = 0; i < 16; i++)
        for (int j = i+1; j < 16; j++) {
            float avg = 0.5f * (mCov[i][j] + mCov[j][i]);
            mCov[i][j] = mCov[j][i] = avg;
        }
}

// ── feedImu ───────────────────────────────────────────────────────────────────
// Accumulates NED-frame gyro (rad/s) and accel (m/s²) during ACCEL_AVG phase.
// After INIT_ACCEL_SECS × 100 Hz samples:
//   Gyro avg → mBiasGyro   Accel avg → roll/pitch via gravity vector
//   Transitions to AWAIT_YAW with yaw=0.
void KalmanFilter::feedImu(float gxNed, float gyNed, float gzNed,
                            float axNed, float ayNed, float azNed) {
    if (mInitPhase != InitPhase::ACCEL_AVG) return;

    mAccumGx += (double)gxNed; mAccumGy += (double)gyNed; mAccumGz += (double)gzNed;
    mAccumAx += (double)axNed; mAccumAy += (double)ayNed; mAccumAz += (double)azNed;
    mAccumCount++;

    const uint32_t target = (uint32_t)(INIT_ACCEL_SECS * 100.0f);
    if (mAccumCount < target) return;

    // Gyro bias from average (plane stationary during init)
    mBiasGyro[0] = (float)(mAccumGx / mAccumCount);
    mBiasGyro[1] = (float)(mAccumGy / mAccumCount);
    mBiasGyro[2] = (float)(mAccumGz / mAccumCount);

    // Roll/pitch from average gravity vector (body = -accel when stationary, NED frame)
    float ax = (float)(mAccumAx / mAccumCount);
    float ay = (float)(mAccumAy / mAccumCount);
    float az = (float)(mAccumAz / mAccumCount);
    float gx = -ax, gy = -ay, gz = -az;

    mInitRoll  = atan2f(gy, gz);
    mInitPitch = atan2f(-gx, sqrtf(gy*gy + gz*gz));

    setQuatFromEuler(mInitRoll, mInitPitch, 0.0f);  // yaw=0 until GPS
    mInitPhase = InitPhase::AWAIT_YAW;
}

// ── feedGpsHeading ────────────────────────────────────────────────────────────
// Captures GPS course-over-ground as initial yaw once speed threshold is met.
// Resets position/velocity to zero on transition — dead-reckoning during
// AWAIT_YAW is unreliable (no yaw, IMU drift) so discard it and let GPS/baro
// pull the estimates to correct values immediately after READY.
void KalmanFilter::feedGpsHeading(float speedMs, float headingDeg) {
    if (mInitPhase != InitPhase::AWAIT_YAW) return;

    mAwaitYawCount++;

    // Accept GPS heading if speed is reliable, OR force transition after 30 s
    // (3000 samples at 100 Hz).  On timeout use whatever heading is available;
    // the GPS yaw update in updateFromGPS will refine it once the plane moves.
    const uint32_t AWAIT_YAW_TIMEOUT = 30u * 100u;
    bool timedOut = (mAwaitYawCount >= AWAIT_YAW_TIMEOUT);

    if (speedMs < GPS_YAW_SPEED_MS && !timedOut) return;

    float yawRad = headingDeg * (3.14159265f / 180.0f);

    // Discard accumulated dead-reckoning: it used yaw=0 and drifted.
    // GPS/baro updates will anchor position and altitude within one cycle.
    mPos[0] = mPos[1] = mPos[2] = 0.0f;
    mVel[0] = mVel[1] = mVel[2] = 0.0f;

    // Reset pos/vel covariance to large values so GPS can immediately correct.
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 16; j++)
            mCov[i][j] = mCov[j][i] = 0.0f;
    mCov[0][0] = mCov[1][1] = mCov[2][2] = 100.0f;  // 10 m std each axis
    mCov[3][3] = mCov[4][4] = mCov[5][5] =   1.0f;  //  1 m/s std each axis

    setQuatFromEuler(mInitRoll, mInitPitch, yawRad);
    mInitPhase = InitPhase::READY;
}

// ── predict ───────────────────────────────────────────────────────────────────
// Integrates quaternion, dead-reckons pos/vel, and propagates error covariance.
// Call every IMU cycle; no-op during ACCEL_AVG or AWAIT_YAW.
void KalmanFilter::predict(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt) {
    if (mInitPhase != InitPhase::READY) return;   // only run when fully initialised
    if (dt <= 0.0f || dt > 1.0f) return;

    // ── Bias-corrected measurements ────────────────────────────────────────
    float wx  = gx - mBiasGyro[0];
    float wy  = gy - mBiasGyro[1];
    float wz  = gz - mBiasGyro[2];
    float acx = ax - mBiasAccel[0];
    float acy = ay - mBiasAccel[1];
    float acz = az - mBiasAccel[2];

    // ── Quaternion kinematics: dq/dt = 0.5 * q ⊗ [0, wx, wy, wz] ─────────
    float qw = mQuat[0], qx = mQuat[1], qy = mQuat[2], qz = mQuat[3];
    mQuat[0] += 0.5f * (-qx*wx - qy*wy - qz*wz) * dt;
    mQuat[1] += 0.5f * ( qw*wx + qy*wz - qz*wy) * dt;
    mQuat[2] += 0.5f * ( qw*wy - qx*wz + qz*wx) * dt;
    mQuat[3] += 0.5f * ( qw*wz + qx*wy - qy*wx) * dt;
    normalizeQuat();

    // ── Body-to-NED DCM from updated quaternion (Sola eq.115) ─────────────
    float R[3][3];
    buildDCM(R);

    // ── Specific force → NED acceleration (gravity in NED = [0,0,+9.81]) ──
    static constexpr float kG = 9.81f;
    float aN = R[0][0]*acx + R[0][1]*acy + R[0][2]*acz;
    float aE = R[1][0]*acx + R[1][1]*acy + R[1][2]*acz;
    float aD = R[2][0]*acx + R[2][1]*acy + R[2][2]*acz + kG;

    // ── Position and velocity integration ─────────────────────────────────
    mPos[0] += mVel[0]*dt + 0.5f*aN*dt*dt;
    mPos[1] += mVel[1]*dt + 0.5f*aE*dt*dt;
    mPos[2] += mVel[2]*dt + 0.5f*aD*dt*dt;
    mVel[0] += aN*dt;
    mVel[1] += aE*dt;
    mVel[2] += aD*dt;

    // ── Error-state covariance propagation ────────────────────────────────
    // Continuous-time Jacobian Fc (Sola §5.4 / eq.269):
    //   δp_dot = δv                       → Fc[0:3, 3:6] = I
    //   δv_dot = -R·[ac]×·δθ - R·δab     → Fc[3:6, 6:9] = -R·[ac]×, Fc[3:6, 9:12] = -R
    //   δθ_dot = -[wc]×·δθ - δwb         → Fc[6:9, 6:9] = -[wc]×,   Fc[6:9, 12:15] = -I
    //   δab_dot = δwb_dot = δb_dot = 0
    //
    // First-order discrete: P += (Fc·P + P·Fc^T)·dt + Qd

    // -R·[ac]× block:
    float Mvt[3][3];
    for (int i = 0; i < 3; i++) {
        Mvt[i][0] = -(R[i][1]*acz - R[i][2]*acy);
        Mvt[i][1] = -(-R[i][0]*acz + R[i][2]*acx);
        Mvt[i][2] = -(R[i][0]*acy - R[i][1]*acx);
    }

    // -[wc]× block:
    float Mtt[3][3] = {
        { 0.0f,  wz, -wy},
        {  -wz, 0.0f,  wx},
        {   wy, -wx, 0.0f}
    };

    float dP[16][16] = {};

    // Fc[δp, δv] = I
    for (int j = 0; j < 16; j++) {
        dP[0][j] += mCov[3][j];
        dP[1][j] += mCov[4][j];
        dP[2][j] += mCov[5][j];
    }

    // Fc[δv, δθ] = Mvt, Fc[δv, δab] = -R
    for (int j = 0; j < 16; j++) {
        for (int k = 0; k < 3; k++) {
            dP[3][j] += Mvt[0][k] * mCov[6+k][j] - R[0][k] * mCov[9+k][j];
            dP[4][j] += Mvt[1][k] * mCov[6+k][j] - R[1][k] * mCov[9+k][j];
            dP[5][j] += Mvt[2][k] * mCov[6+k][j] - R[2][k] * mCov[9+k][j];
        }
    }

    // Fc[δθ, δθ] = Mtt, Fc[δθ, δwb] = -I
    for (int j = 0; j < 16; j++) {
        for (int k = 0; k < 3; k++) {
            dP[6][j] += Mtt[0][k] * mCov[6+k][j];
            dP[7][j] += Mtt[1][k] * mCov[6+k][j];
            dP[8][j] += Mtt[2][k] * mCov[6+k][j];
        }
        dP[6][j] -= mCov[12][j];
        dP[7][j] -= mCov[13][j];
        dP[8][j] -= mCov[14][j];
    }

    for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++)
            mCov[i][j] += (dP[i][j] + dP[j][i]) * dt;

    // Additive discrete process noise Qd (diagonal)
    float qa2  = SIGMA_ACCEL      * SIGMA_ACCEL      * dt * dt;  // Vi = σ²_ãn·Δt²·I  (Solà eq.262)
    float qw2  = SIGMA_GYRO       * SIGMA_GYRO       * dt * dt;  // Θi = σ²_ω̃n·Δt²·I (Solà eq.263)
    float qab2 = SIGMA_ACCEL_WALK * SIGMA_ACCEL_WALK * dt;
    float qwb2 = SIGMA_GYRO_WALK  * SIGMA_GYRO_WALK  * dt;
    float qbb2 = SIGMA_BARO_WALK  * SIGMA_BARO_WALK  * dt;

    mCov[3][3]   += qa2;  mCov[4][4]   += qa2;  mCov[5][5]   += qa2;
    mCov[6][6]   += qw2;  mCov[7][7]   += qw2;  mCov[8][8]   += qw2;
    mCov[9][9]   += qab2; mCov[10][10] += qab2; mCov[11][11] += qab2;
    mCov[12][12] += qwb2; mCov[13][13] += qwb2; mCov[14][14] += qwb2;
    mCov[15][15] += qbb2;

    symmetrizeCov();

    // Decouple attitude from all non-attitude states.
    // P[att, pos/vel/biases] builds up via the Fc Jacobian and causes GPS
    // position/velocity and barometer to spuriously rotate the attitude estimate.
    // Zeroing these cross-covariances every predict() step keeps them at zero,
    // so K[att] is always 0 for GPS and baro — attitude is only updated by the
    // explicit GPS heading measurement (Part 2 of updateFromGPS).
    // P[att, att] (the 3×3 diagonal block) is preserved so the yaw update works.
    for (int j = 0; j < 16; j++) {
        if (j >= 6 && j <= 8) continue;    // keep the att×att block
        mCov[6][j] = mCov[j][6] = 0.0f;
        mCov[7][j] = mCov[j][7] = 0.0f;
        mCov[8][j] = mCov[j][8] = 0.0f;
    }
}

// ── injectErrorState ──────────────────────────────────────────────────────────
// Adds error-state correction dx[16] to nominal state.
// Attitude uses multiplicative quaternion update (small-angle rotation).
void KalmanFilter::injectErrorState(const float dx[16]) {
    mPos[0] += dx[0]; mPos[1] += dx[1]; mPos[2] += dx[2];
    mVel[0] += dx[3]; mVel[1] += dx[4]; mVel[2] += dx[5];

    // Multiplicative attitude update: q ← q ⊗ δq, δq ≈ [1, δθ/2]
    float dqw = 1.0f, dqx = 0.5f*dx[6], dqy = 0.5f*dx[7], dqz = 0.5f*dx[8];
    float w = mQuat[0], x = mQuat[1], y = mQuat[2], z = mQuat[3];
    mQuat[0] = w*dqw - x*dqx - y*dqy - z*dqz;
    mQuat[1] = w*dqx + x*dqw + y*dqz - z*dqy;
    mQuat[2] = w*dqy - x*dqz + y*dqw + z*dqx;
    mQuat[3] = w*dqz + x*dqy - y*dqx + z*dqw;
    normalizeQuat();

    mBiasAccel[0] += dx[9];  mBiasAccel[1] += dx[10]; mBiasAccel[2] += dx[11];
    mBiasGyro[0]  += dx[12]; mBiasGyro[1]  += dx[13]; mBiasGyro[2]  += dx[14];
    mBaroBias     += dx[15];
}

// ── mat5Invert ────────────────────────────────────────────────────────────────
// Gauss-Jordan elimination with partial pivoting on a 5×5 matrix.
// Returns false if matrix is singular (pivot < 1e-12).
bool KalmanFilter::mat5Invert(float A[5][5], float Ainv[5][5]) {
    float aug[5][10];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) aug[i][j] = A[i][j];
        for (int j = 0; j < 5; j++) aug[i][5+j] = (i == j) ? 1.0f : 0.0f;
    }

    for (int col = 0; col < 5; col++) {
        int pivot = col;
        float best = fabsf(aug[col][col]);
        for (int row = col+1; row < 5; row++) {
            float v = fabsf(aug[row][col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best < 1e-12f) return false;

        if (pivot != col) {
            for (int j = 0; j < 10; j++) {
                float tmp = aug[col][j]; aug[col][j] = aug[pivot][j]; aug[pivot][j] = tmp;
            }
        }

        float inv = 1.0f / aug[col][col];
        for (int j = 0; j < 10; j++) aug[col][j] *= inv;

        for (int row = 0; row < 5; row++) {
            if (row == col) continue;
            float f = aug[row][col];
            for (int j = 0; j < 10; j++) aug[row][j] -= f * aug[col][j];
        }
    }

    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            Ainv[i][j] = aug[i][5+j];
    return true;
}

// ── updateFromGPS ─────────────────────────────────────────────────────────────
// 5-measurement update: posN, posE, posD, velN, velE.
// H (5×16): identity block selecting state indices 0..4 (δp + δv_NE).
// Followed by optional 1-measurement yaw update from course-over-ground.
void KalmanFilter::updateFromGPS(float posN, float posE, float posD,
                                  float velN, float velE,
                                  float hdop, float headingDeg, float speedMs) {
    if (mInitPhase != InitPhase::READY) return;

    // ── Part 1: position + velocity update (5D) ────────────────────────────
    // Clamp HDOP to minimum 1.0 so measurement noise never collapses to zero.
    float hdopEff = (hdop < 1.0f) ? 1.0f : hdop;
    float rp2 = (SIGMA_GPS_POS * hdopEff) * (SIGMA_GPS_POS * hdopEff);
    float rv2 = (SIGMA_GPS_VEL * hdopEff) * (SIGMA_GPS_VEL * hdopEff);

    // Innovation: measured - nominal
    float innov[5] = {
        posN - mPos[0], posE - mPos[1], posD - mPos[2],
        velN - mVel[0], velE - mVel[1]
    };

    // S = H·P·H^T + R  →  top-left 5×5 of P plus diagonal measurement noise.
    // Save H·P (rows 0..4 of P) BEFORE modifying P to prevent read-back corruption.
    float HP[5][16];
    for (int m = 0; m < 5; m++)
        for (int j = 0; j < 16; j++)
            HP[m][j] = mCov[m][j];

    float S[5][5];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            S[i][j] = mCov[i][j];
    S[0][0] += rp2; S[1][1] += rp2; S[2][2] += rp2;
    S[3][3] += rv2; S[4][4] += rv2;

    float Sinv[5][5];
    if (!mat5Invert(S, Sinv)) return;

    // K = P·H^T · Sinv  (16×5) — P·H^T is just the first 5 columns of P.
    float K[16][5];
    for (int i = 0; i < 16; i++)
        for (int m = 0; m < 5; m++) {
            float s = 0.0f;
            for (int k = 0; k < 5; k++) s += mCov[i][k] * Sinv[k][m];
            K[i][m] = s;
        }

    // dx = K · innov
    float dx[16] = {};
    for (int i = 0; i < 16; i++)
        for (int m = 0; m < 5; m++)
            dx[i] += K[i][m] * innov[m];

    // P = P - K·(H·P)  — use pre-saved HP to avoid read-back corruption.
    for (int i = 0; i < 16; i++)
        for (int j = 0; j < 16; j++) {
            float kHP = 0.0f;
            for (int m = 0; m < 5; m++) kHP += K[i][m] * HP[m][j];
            mCov[i][j] -= kHP;
        }
    symmetrizeCov();

    injectErrorState(dx);

    // ── Part 2: yaw update from GPS course-over-ground (1D) ───────────────
    if (speedMs < GPS_HDG_SPEED_MS) return;

    float yawRad = headingDeg * (3.14159265f / 180.0f);
    float nomYaw = atan2f(2.0f*(mQuat[0]*mQuat[3] + mQuat[1]*mQuat[2]),
                          1.0f - 2.0f*(mQuat[2]*mQuat[2] + mQuat[3]*mQuat[3]));
    float dPsi = yawRad - nomYaw;
    while (dPsi >  3.14159265f) dPsi -= 6.28318530f;
    while (dPsi < -3.14159265f) dPsi += 6.28318530f;

    // H_yaw: index 8 (δθ_z).  Save row 8 before modifying P.
    float ryaw = SIGMA_GPS_YAW * (3.14159265f / 180.0f);
    float Syaw = mCov[8][8] + ryaw*ryaw;
    if (Syaw < 1e-9f) return;

    float P8[16];
    for (int j = 0; j < 16; j++) P8[j] = mCov[8][j];

    float dxYaw[16] = {};
    for (int i = 0; i < 16; i++) {
        float Ki = mCov[i][8] / Syaw;
        dxYaw[i] = Ki * dPsi;
        for (int j = 0; j < 16; j++) mCov[i][j] -= Ki * P8[j];
    }
    symmetrizeCov();

    injectErrorState(dxYaw);
}

// ── updateFromBarometer ───────────────────────────────────────────────────────
// 1-measurement update: altM = altitude above ground, positive-up (m).
// Nominal output: alt = -posD + baroBias.
// H (1×16): index 2 → -1, index 15 → +1.
void KalmanFilter::updateFromBarometer(float altM) {
    if (mInitPhase != InitPhase::READY) return;

    float nomAlt = -mPos[2] + mBaroBias;
    float innov  = altM - nomAlt;

    // S = H·P·H^T + R
    float Sbaro = mCov[2][2] - mCov[2][15] - mCov[15][2] + mCov[15][15]
                  + SIGMA_BARO_MEAS * SIGMA_BARO_MEAS;
    if (Sbaro < 1e-9f) return;

    // Save H·P = -P[2,:] + P[15,:] before the update.
    float HP_baro[16];
    for (int j = 0; j < 16; j++) HP_baro[j] = -mCov[2][j] + mCov[15][j];

    float dxBaro[16] = {};
    for (int i = 0; i < 16; i++) {
        float Ki = (-mCov[i][2] + mCov[i][15]) / Sbaro;
        dxBaro[i] = Ki * innov;
        for (int j = 0; j < 16; j++) mCov[i][j] -= Ki * HP_baro[j];
    }
    symmetrizeCov();

    injectErrorState(dxBaro);
}

// ── Getters ───────────────────────────────────────────────────────────────────
void KalmanFilter::getQuaternion(float &w, float &x, float &y, float &z) const {
    w = mQuat[0]; x = mQuat[1]; y = mQuat[2]; z = mQuat[3];
}

void KalmanFilter::getEulerDeg(float &roll, float &pitch, float &yaw) const {
    float w = mQuat[0], x = mQuat[1], y = mQuat[2], z = mQuat[3];

    roll = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * (180.0f / 3.14159265f);

    float sinp = 2.0f*(w*y - z*x);
    sinp  = (sinp >  1.0f) ?  1.0f : ((sinp < -1.0f) ? -1.0f : sinp);
    pitch = asinf(sinp) * (180.0f / 3.14159265f);

    yaw = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)) * (180.0f / 3.14159265f);
}

void KalmanFilter::getPosition(float &posN, float &posE, float &posD) const {
    posN = mPos[0]; posE = mPos[1]; posD = mPos[2];
}

void KalmanFilter::getVelocity(float &velN, float &velE, float &velD) const {
    velN = mVel[0]; velE = mVel[1]; velD = mVel[2];
}

void KalmanFilter::getGyroBias(float &bgx, float &bgy, float &bgz) const {
    bgx = mBiasGyro[0]; bgy = mBiasGyro[1]; bgz = mBiasGyro[2];
}
