#include "KalmanFilter.h"
#include <string.h>  // memset

// ── reset ─────────────────────────────────────────────────────────────────────
void KalmanFilter::reset() {
    mPos[0] = mPos[1] = mPos[2] = 0.0f;
    mVel[0] = mVel[1] = mVel[2] = 0.0f;
    mQuat[0] = 1.0f; mQuat[1] = mQuat[2] = mQuat[3] = 0.0f;
    mBiasAccel[0] = mBiasAccel[1] = mBiasAccel[2] = 0.0f;
    mBiasGyro[0]  = mBiasGyro[1]  = mBiasGyro[2]  = 0.0f;
    mBaroBias     = 0.0f;
    mGpsBiasPos[0] = mGpsBiasPos[1] = mGpsBiasPos[2] = 0.0f;
    mGpsBiasVel[0] = mGpsBiasVel[1] = 0.0f;

    memset(mCov, 0, sizeof(mCov));

    // Initial diagonal covariance (21-D error state)
    // [δp δv δθ δab δwb δbaro δbgp δbgv]
    const float diag[21] = {
        1.0f,   1.0f,   1.0f,    // δp  — ~1 m uncertainty from GPS origin average
        0.01f,  0.01f,  0.01f,   // δv  — ~0.1 m/s uncertainty at rest
        0.1f,   0.1f,   1.0f,    // δθ  — roll/pitch small, yaw unknown
        1e-4f,  1e-4f,  1e-4f,   // δab — small initial accel bias uncertainty
        1e-6f,  1e-6f,  1e-6f,   // δwb — small initial gyro bias uncertainty
        1.0f,                    // δbb — baro bias unknown (1 m std)
        1.0f,   1.0f,   1.0f,    // δbgp — GPS pos bias (1 m std, not 5 m)
        0.01f,  0.01f            // δbgv — GPS vel bias (0.1 m/s std)
    };
    for (int i = 0; i < 21; i++) mCov[i][i] = diag[i];

    mInitPhase  = InitPhase::ACCEL_AVG;
    mAccumGx    = 0; mAccumGy = 0; mAccumGz = 0;
    mAccumAx    = 0; mAccumAy = 0; mAccumAz = 0;
    mAccumCount = 0;
    mInitRoll   = 0.0f;
    mInitPitch  = 0.0f;
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
    for (int i = 0; i < 21; i++)
        for (int j = i+1; j < 21; j++) {
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
    float gx = -ax, gy = -ay, gz = -az;  // gravity in body frame

    mInitRoll  = atan2f(gy, gz);
    mInitPitch = atan2f(-gx, sqrtf(gy*gy + gz*gz));

    setQuatFromEuler(mInitRoll, mInitPitch, 0.0f);  // yaw=0 until GPS
    mInitPhase = InitPhase::AWAIT_YAW;
}

// ── feedGpsHeading ────────────────────────────────────────────────────────────
// Captures GPS course-over-ground as initial yaw once speed threshold is met.
// Rotates the accumulated dead-reckoned pos/vel into true NED via R_z(yaw).
void KalmanFilter::feedGpsHeading(float speedMs, float headingDeg) {
    if (mInitPhase != InitPhase::AWAIT_YAW) return;
    if (speedMs < GPS_YAW_SPEED_MS) return;

    float yawRad = headingDeg * (3.14159265f / 180.0f);

    // Yaw-snap: rotate accumulated pos/vel from yaw=0 frame into true NED
    float cy = cosf(yawRad), sy = sinf(yawRad);
    float pN = cy*mPos[0] - sy*mPos[1]; float pE = sy*mPos[0] + cy*mPos[1];
    mPos[0] = pN; mPos[1] = pE;
    float vN = cy*mVel[0] - sy*mVel[1]; float vE = sy*mVel[0] + cy*mVel[1];
    mVel[0] = vN; mVel[1] = vE;

    setQuatFromEuler(mInitRoll, mInitPitch, yawRad);
    mInitPhase = InitPhase::READY;
}

// ── predict ───────────────────────────────────────────────────────────────────
// Integrates quaternion, dead-reckons pos/vel, and propagates error covariance.
// Call every IMU cycle after feedImu(); no-op during ACCEL_AVG.
void KalmanFilter::predict(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt) {
    if (mInitPhase == InitPhase::ACCEL_AVG) return;
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

    // ── Position and velocity integration (trapezoid) ──────────────────────
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
    //   δbgp_dot = δbgv_dot = 0          (GPS bias: pure random walks, no coupling)
    //
    // First-order discrete update: P += (Fc·P + P·Fc^T)·dt + Qd

    // -R·[ac]× block (Sola eq.269, δv/δθ coupling):
    float Mvt[3][3];
    for (int i = 0; i < 3; i++) {
        Mvt[i][0] = -(R[i][1]*acz - R[i][2]*acy);
        Mvt[i][1] = -(-R[i][0]*acz + R[i][2]*acx);
        Mvt[i][2] = -(R[i][0]*acy - R[i][1]*acx);
    }

    // -[wc]× block (δθ/δθ coupling):
    float Mtt[3][3] = {
        { 0.0f,  wz, -wy},
        {  -wz, 0.0f,  wx},
        {  wy, -wx, 0.0f}
    };

    // Accumulate dP = Fc·P using sparse row blocks of Fc.
    // GPS bias rows (16-20) have zero Fc entries — no contribution.
    float dP[21][21] = {};

    // Fc block [δp, δv] = I  →  dP[0:3, :] += P[3:6, :]
    for (int j = 0; j < 21; j++) {
        dP[0][j] += mCov[3][j];
        dP[1][j] += mCov[4][j];
        dP[2][j] += mCov[5][j];
    }

    // Fc block [δv, δθ] = Mvt, [δv, δab] = -R  →  dP[3:6, :]
    for (int j = 0; j < 21; j++) {
        for (int k = 0; k < 3; k++) {
            dP[3][j] += Mvt[0][k] * mCov[6+k][j] - R[0][k] * mCov[9+k][j];
            dP[4][j] += Mvt[1][k] * mCov[6+k][j] - R[1][k] * mCov[9+k][j];
            dP[5][j] += Mvt[2][k] * mCov[6+k][j] - R[2][k] * mCov[9+k][j];
        }
    }

    // Fc block [δθ, δθ] = Mtt, [δθ, δwb] = -I  →  dP[6:9, :]
    for (int j = 0; j < 21; j++) {
        for (int k = 0; k < 3; k++) {
            dP[6][j] += Mtt[0][k] * mCov[6+k][j];
            dP[7][j] += Mtt[1][k] * mCov[6+k][j];
            dP[8][j] += Mtt[2][k] * mCov[6+k][j];
        }
        dP[6][j] -= mCov[12][j];
        dP[7][j] -= mCov[13][j];
        dP[8][j] -= mCov[14][j];
    }

    // Apply P += (dP + dP^T) * dt  (dP^T is the P*Fc^T term)
    for (int i = 0; i < 21; i++)
        for (int j = 0; j < 21; j++)
            mCov[i][j] += (dP[i][j] + dP[j][i]) * dt;

    // Additive discrete process noise Qd (diagonal):
    float qa2   = SIGMA_ACCEL        * SIGMA_ACCEL        * dt;
    float qw2   = SIGMA_GYRO         * SIGMA_GYRO         * dt;
    float qab2  = SIGMA_ACCEL_WALK   * SIGMA_ACCEL_WALK   * dt;
    float qwb2  = SIGMA_GYRO_WALK    * SIGMA_GYRO_WALK    * dt;
    float qbb2  = SIGMA_BARO_WALK    * SIGMA_BARO_WALK    * dt;
    float qgpp2 = SIGMA_GPS_POS_WALK * SIGMA_GPS_POS_WALK * dt;
    float qgpv2 = SIGMA_GPS_VEL_WALK * SIGMA_GPS_VEL_WALK * dt;

    mCov[3][3]   += qa2;   mCov[4][4]   += qa2;   mCov[5][5]   += qa2;
    mCov[6][6]   += qw2;   mCov[7][7]   += qw2;   mCov[8][8]   += qw2;
    mCov[9][9]   += qab2;  mCov[10][10] += qab2;  mCov[11][11] += qab2;
    mCov[12][12] += qwb2;  mCov[13][13] += qwb2;  mCov[14][14] += qwb2;
    mCov[15][15] += qbb2;
    mCov[16][16] += qgpp2; mCov[17][17] += qgpp2; mCov[18][18] += qgpp2;
    mCov[19][19] += qgpv2; mCov[20][20] += qgpv2;

    symmetrizeCov();
}

// ── injectErrorState ──────────────────────────────────────────────────────────
// Adds error-state correction dx[21] to nominal state.
// Attitude uses multiplicative quaternion update (small-angle rotation).
void KalmanFilter::injectErrorState(const float dx[21]) {
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

    mBiasAccel[0]  += dx[9];  mBiasAccel[1]  += dx[10]; mBiasAccel[2]  += dx[11];
    mBiasGyro[0]   += dx[12]; mBiasGyro[1]   += dx[13]; mBiasGyro[2]   += dx[14];
    mBaroBias      += dx[15];
    mGpsBiasPos[0] += dx[16]; mGpsBiasPos[1] += dx[17]; mGpsBiasPos[2] += dx[18];
    mGpsBiasVel[0] += dx[19]; mGpsBiasVel[1] += dx[20];
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
// H (5×21): each measurement touches one state column + one GPS-bias column:
//   m=0..2  posN/E/D:  col ca[m]= m,    col cb[m]= 16+m  (δp + δbgp)
//   m=3..4  velN/E:    col ca[m]= m,    col cb[m]= 19+m-3 (δv + δbgv)
// Followed by optional 1-measurement yaw update from course-over-ground.
void KalmanFilter::updateFromGPS(float posN, float posE, float posD,
                                  float velN, float velE,
                                  float hdop, float headingDeg, float speedMs) {
    if (mInitPhase != InitPhase::READY) return;

    // ── Part 1: position + velocity update (5D) ────────────────────────────
    // Clamp HDOP to minimum 1.0: prevents rp2=rv2=0 when GGA not yet received,
    // which would drive GPS-bias covariance to 0 and make S singular on next update.
    float hdopEff = (hdop < 1.0f) ? 1.0f : hdop;
    float rp = SIGMA_GPS_POS * hdopEff;
    float rv = SIGMA_GPS_VEL * hdopEff;
    float rp2 = rp*rp, rv2 = rv*rv;

    // Column-index pairs for sparse H:  H[m, ca[m]]=1, H[m, cb[m]]=1
    const int ca[5] = {0, 1, 2, 3, 4};          // δp_N/E/D, δv_N/E
    const int cb[5] = {16, 17, 18, 19, 20};      // δbgp_N/E/D, δbgv_N/E

    // Innovation: measurement - (nominal state + GPS bias estimate)
    float innov[5] = {
        posN - mPos[0] - mGpsBiasPos[0],
        posE - mPos[1] - mGpsBiasPos[1],
        posD - mPos[2] - mGpsBiasPos[2],
        velN - mVel[0] - mGpsBiasVel[0],
        velE - mVel[1] - mGpsBiasVel[1]
    };

    // HP[m][j] = (H·P)[m][j] = P[ca[m]][j] + P[cb[m]][j]
    // Saved BEFORE any P modification to avoid read-back corruption.
    float HP[5][21];
    for (int m = 0; m < 5; m++)
        for (int j = 0; j < 21; j++)
            HP[m][j] = mCov[ca[m]][j] + mCov[cb[m]][j];

    // S = H·P·H^T + R  (5×5)
    // S[m1][m2] = HP[m1][ca[m2]] + HP[m1][cb[m2]]
    float S[5][5];
    for (int m1 = 0; m1 < 5; m1++)
        for (int m2 = 0; m2 < 5; m2++)
            S[m1][m2] = HP[m1][ca[m2]] + HP[m1][cb[m2]];
    S[0][0] += rp2; S[1][1] += rp2; S[2][2] += rp2;
    S[3][3] += rv2; S[4][4] += rv2;

    float Sinv[5][5];
    if (!mat5Invert(S, Sinv)) return;

    // PHT[i][m] = (P·H^T)[i][m] = P[i][ca[m]] + P[i][cb[m]]
    float PHT[21][5];
    for (int i = 0; i < 21; i++)
        for (int m = 0; m < 5; m++)
            PHT[i][m] = mCov[i][ca[m]] + mCov[i][cb[m]];

    // K = PHT · Sinv  (21×5)
    float K[21][5];
    for (int i = 0; i < 21; i++)
        for (int m = 0; m < 5; m++) {
            float s = 0.0f;
            for (int k = 0; k < 5; k++) s += PHT[i][k] * Sinv[k][m];
            K[i][m] = s;
        }

    // dx = K · innov  (21×1)
    float dx[21] = {};
    for (int i = 0; i < 21; i++)
        for (int m = 0; m < 5; m++)
            dx[i] += K[i][m] * innov[m];

    // P = P - K·(H·P) = P - K·HP
    for (int i = 0; i < 21; i++)
        for (int j = 0; j < 21; j++) {
            float kHP = 0.0f;
            for (int m = 0; m < 5; m++) kHP += K[i][m] * HP[m][j];
            mCov[i][j] -= kHP;
        }
    symmetrizeCov();

    injectErrorState(dx);

    // ── Part 2: yaw update from GPS course-over-ground (1D) ───────────────
    if (speedMs < GPS_YAW_SPEED_MS) return;

    float yawRad = headingDeg * (3.14159265f / 180.0f);
    float nomYaw = atan2f(2.0f*(mQuat[0]*mQuat[3] + mQuat[1]*mQuat[2]),
                          1.0f - 2.0f*(mQuat[2]*mQuat[2] + mQuat[3]*mQuat[3]));
    float dPsi = yawRad - nomYaw;
    while (dPsi >  3.14159265f) dPsi -= 6.28318530f;
    while (dPsi < -3.14159265f) dPsi += 6.28318530f;

    // H_yaw (1×21): index 8 (δθ_z) only
    float ryaw = SIGMA_GPS_YAW * (3.14159265f / 180.0f);
    float Syaw = mCov[8][8] + ryaw*ryaw;
    if (Syaw < 1e-9f) return;

    // Save H·P = row 8 of P before the update.
    float P8[21];
    for (int j = 0; j < 21; j++) P8[j] = mCov[8][j];

    float dxYaw[21] = {};
    for (int i = 0; i < 21; i++) {
        float Ki = mCov[i][8] / Syaw;
        dxYaw[i] = Ki * dPsi;
        for (int j = 0; j < 21; j++) mCov[i][j] -= Ki * P8[j];
    }
    symmetrizeCov();

    injectErrorState(dxYaw);
}

// ── updateFromBarometer ───────────────────────────────────────────────────────
// 1-measurement update: altM = altitude above ground, positive-up (m).
// Nominal output: alt = -posD + baroBias.
// H (1×21): index 2 → -1, index 15 → +1.
void KalmanFilter::updateFromBarometer(float altM) {
    if (mInitPhase != InitPhase::READY) return;

    float nomAlt = -mPos[2] + mBaroBias;
    float innov  = altM - nomAlt;

    // S = H·P·H^T + R = P[2,2] - P[2,15] - P[15,2] + P[15,15] + σ²
    float Sbaro = mCov[2][2] - mCov[2][15] - mCov[15][2] + mCov[15][15]
                  + SIGMA_BARO_MEAS * SIGMA_BARO_MEAS;
    if (Sbaro < 1e-9f) return;

    // Save H·P = -P[2,:] + P[15,:] before the update to prevent read-back corruption.
    float HP_baro[21];
    for (int j = 0; j < 21; j++) HP_baro[j] = -mCov[2][j] + mCov[15][j];

    float dxBaro[21] = {};
    for (int i = 0; i < 21; i++) {
        float Ki = (-mCov[i][2] + mCov[i][15]) / Sbaro;
        dxBaro[i] = Ki * innov;
        for (int j = 0; j < 21; j++) mCov[i][j] -= Ki * HP_baro[j];
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

void KalmanFilter::getGpsBiasPos(float &bN, float &bE, float &bD) const {
    bN = mGpsBiasPos[0]; bE = mGpsBiasPos[1]; bD = mGpsBiasPos[2];
}

void KalmanFilter::getGpsBiasVel(float &bN, float &bE) const {
    bN = mGpsBiasVel[0]; bE = mGpsBiasVel[1];
}
