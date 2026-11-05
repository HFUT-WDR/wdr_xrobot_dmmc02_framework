#include "dm_imu_l1_attitude.h"

#include <math.h>
#include <string.h>

#include "Kalman_Filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_PER_RAD (57.2957795130823208768f)

static const float k_stable_gyro_limit = 0.3f;
static const float k_stable_accel_limit = 0.5f;
static const float k_gravity_accel = 9.80665f;
static const float k_chi_threshold = 1e-8f;

typedef struct
{
    uint8_t active;

    float q1;
    float q2;
    float r;
    float lambda;
    float acc_lpf_coef;

    float accel_inv_norm;
    float gyro_inv_norm;
    float half_gyro_dt[3];
    float accel_lpf[3];

    float matrix_a_init[36];
    float matrix_p_init[36];

    KalmanFilter_Info_TypeDef ekf;
    dm_imu_l1_attitude_t output;
} dm_imu_l1_attitude_ctx_t;

static dm_imu_l1_attitude_ctx_t s_ctx[DM_IMU_L1_ATTITUDE_IMU_COUNT];

static float dm_imu_l1_attitude_inv_sqrt(float x);
static void dm_imu_l1_attitude_fill_default_matrix(dm_imu_l1_attitude_ctx_t *ctx);
static dm_imu_l1_attitude_ctx_t *dm_imu_l1_attitude_get_ctx_from_kf(KalmanFilter_Info_TypeDef *kf);

static void dm_imu_l1_attitude_a_update(KalmanFilter_Info_TypeDef *kf);
static void dm_imu_l1_attitude_h_update(KalmanFilter_Info_TypeDef *kf);
static uint8_t dm_imu_l1_attitude_chi_sqrt_test(KalmanFilter_Info_TypeDef *kf, dm_imu_l1_attitude_ctx_t *ctx);
static void dm_imu_l1_attitude_xhat_update(KalmanFilter_Info_TypeDef *kf);

static float dm_imu_l1_attitude_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void dm_imu_l1_attitude_fill_default_matrix(dm_imu_l1_attitude_ctx_t *ctx)
{
    memset(ctx->matrix_a_init, 0, sizeof(ctx->matrix_a_init));
    memset(ctx->matrix_p_init, 0, sizeof(ctx->matrix_p_init));

    ctx->matrix_a_init[0] = 1.0f;
    ctx->matrix_a_init[7] = 1.0f;
    ctx->matrix_a_init[14] = 1.0f;
    ctx->matrix_a_init[21] = 1.0f;
    ctx->matrix_a_init[28] = 1.0f;
    ctx->matrix_a_init[35] = 1.0f;

    ctx->matrix_p_init[0] = 100000.0f;
    ctx->matrix_p_init[1] = 0.1f;
    ctx->matrix_p_init[2] = 0.1f;
    ctx->matrix_p_init[3] = 0.1f;
    ctx->matrix_p_init[4] = 0.1f;
    ctx->matrix_p_init[5] = 0.1f;

    ctx->matrix_p_init[6] = 0.1f;
    ctx->matrix_p_init[7] = 100000.0f;
    ctx->matrix_p_init[8] = 0.1f;
    ctx->matrix_p_init[9] = 0.1f;
    ctx->matrix_p_init[10] = 0.1f;
    ctx->matrix_p_init[11] = 0.1f;

    ctx->matrix_p_init[12] = 0.1f;
    ctx->matrix_p_init[13] = 0.1f;
    ctx->matrix_p_init[14] = 100000.0f;
    ctx->matrix_p_init[15] = 0.1f;
    ctx->matrix_p_init[16] = 0.1f;
    ctx->matrix_p_init[17] = 0.1f;

    ctx->matrix_p_init[18] = 0.1f;
    ctx->matrix_p_init[19] = 0.1f;
    ctx->matrix_p_init[20] = 0.1f;
    ctx->matrix_p_init[21] = 100000.0f;
    ctx->matrix_p_init[22] = 0.1f;
    ctx->matrix_p_init[23] = 0.1f;

    ctx->matrix_p_init[24] = 0.1f;
    ctx->matrix_p_init[25] = 0.1f;
    ctx->matrix_p_init[26] = 0.1f;
    ctx->matrix_p_init[27] = 0.1f;
    ctx->matrix_p_init[28] = 100.0f;
    ctx->matrix_p_init[29] = 0.1f;

    ctx->matrix_p_init[30] = 0.1f;
    ctx->matrix_p_init[31] = 0.1f;
    ctx->matrix_p_init[32] = 0.1f;
    ctx->matrix_p_init[33] = 0.1f;
    ctx->matrix_p_init[34] = 0.1f;
    ctx->matrix_p_init[35] = 100.0f;
}

static dm_imu_l1_attitude_ctx_t *dm_imu_l1_attitude_get_ctx_from_kf(KalmanFilter_Info_TypeDef *kf)
{
    uint8_t i;

    for (i = 0U; i < DM_IMU_L1_ATTITUDE_IMU_COUNT; i++) {
        if (&s_ctx[i].ekf == kf) {
            return &s_ctx[i];
        }
    }

    return NULL;
}

void dm_imu_l1_attitude_reset(uint8_t imu_index)
{
    dm_imu_l1_attitude_ctx_t *ctx;

    if (imu_index >= DM_IMU_L1_ATTITUDE_IMU_COUNT) {
        return;
    }

    ctx = &s_ctx[imu_index];
    memset(ctx, 0, sizeof(*ctx));

    ctx->q1 = DM_IMU_L1_ATTITUDE_PROCESS_NOISE_Q1;
    ctx->q2 = DM_IMU_L1_ATTITUDE_PROCESS_NOISE_Q2;
    ctx->r = DM_IMU_L1_ATTITUDE_MEASURE_NOISE_R;
    ctx->lambda = DM_IMU_L1_ATTITUDE_DEFAULT_LAMBDA;
    ctx->acc_lpf_coef = DM_IMU_L1_ATTITUDE_ACCEL_LPF;

    dm_imu_l1_attitude_fill_default_matrix(ctx);

    Kalman_Filter_Init(&ctx->ekf, 6U, 0U, 3U);
    ctx->ekf.ChiSquareTest.TestFlag = false;
    ctx->ekf.ChiSquareTest.Result = false;
    ctx->ekf.ChiSquareTest.ChiSquareTestThresholds = k_chi_threshold;
    ctx->ekf.ChiSquareTest.ChiSquareCnt = 0U;

    ctx->ekf.Data.Xhat[0] = 1.0f;
    ctx->ekf.Data.Xhat[1] = 0.0f;
    ctx->ekf.Data.Xhat[2] = 0.0f;
    ctx->ekf.Data.Xhat[3] = 0.0f;
    ctx->ekf.Data.Xhat[4] = 0.0f;
    ctx->ekf.Data.Xhat[5] = 0.0f;

    ctx->ekf.User_Function1 = dm_imu_l1_attitude_a_update;
    ctx->ekf.User_Function2 = dm_imu_l1_attitude_h_update;
    ctx->ekf.User_Function3 = dm_imu_l1_attitude_xhat_update;

    ctx->ekf.SkipStep3 = true;
    ctx->ekf.SkipStep4 = true;

    memcpy(ctx->ekf.Data.A, ctx->matrix_a_init, sizeof(ctx->matrix_a_init));
    memcpy(ctx->ekf.Data.P, ctx->matrix_p_init, sizeof(ctx->matrix_p_init));

    ctx->output.quat[0] = 1.0f;
    ctx->output.quat[1] = 0.0f;
    ctx->output.quat[2] = 0.0f;
    ctx->output.quat[3] = 0.0f;
    ctx->active = 1U;
}

void dm_imu_l1_attitude_init_all(void)
{
    uint8_t i;

    for (i = 0U; i < DM_IMU_L1_ATTITUDE_IMU_COUNT; i++) {
        dm_imu_l1_attitude_reset(i);
    }
}

uint8_t dm_imu_l1_attitude_update(uint8_t imu_index, const float gyro[3], const float accel[3], float dt)
{
    dm_imu_l1_attitude_ctx_t *ctx;
    float accel_meas[3];
    float gyro_norm;
    float accel_norm;
    float alpha;
    uint8_t i;

    if (imu_index >= DM_IMU_L1_ATTITUDE_IMU_COUNT || gyro == NULL || accel == NULL || dt <= 0.0f) {
        return 0U;
    }

    ctx = &s_ctx[imu_index];
    if (ctx->active == 0U) {
        dm_imu_l1_attitude_reset(imu_index);
        ctx = &s_ctx[imu_index];
    }

    ctx->ekf.dt = dt;

    ctx->output.gyro[0] = gyro[0] - ctx->output.gyro_bias[0];
    ctx->output.gyro[1] = gyro[1] - ctx->output.gyro_bias[1];
    ctx->output.gyro[2] = gyro[2] - ctx->output.gyro_bias[2];

    ctx->gyro_inv_norm = dm_imu_l1_attitude_inv_sqrt(ctx->output.gyro[0] * ctx->output.gyro[0] +
                                                      ctx->output.gyro[1] * ctx->output.gyro[1] +
                                                      ctx->output.gyro[2] * ctx->output.gyro[2]);

    ctx->half_gyro_dt[0] = 0.5f * ctx->output.gyro[0] * dt;
    ctx->half_gyro_dt[1] = 0.5f * ctx->output.gyro[1] * dt;
    ctx->half_gyro_dt[2] = 0.5f * ctx->output.gyro[2] * dt;

    memcpy(ctx->ekf.Data.A, ctx->matrix_a_init, sizeof(ctx->matrix_a_init));

    ctx->ekf.Data.A[1] = -ctx->half_gyro_dt[0];
    ctx->ekf.Data.A[2] = -ctx->half_gyro_dt[1];
    ctx->ekf.Data.A[3] = -ctx->half_gyro_dt[2];
    ctx->ekf.Data.A[6] = ctx->half_gyro_dt[0];
    ctx->ekf.Data.A[8] = ctx->half_gyro_dt[2];
    ctx->ekf.Data.A[9] = -ctx->half_gyro_dt[1];
    ctx->ekf.Data.A[12] = ctx->half_gyro_dt[1];
    ctx->ekf.Data.A[13] = -ctx->half_gyro_dt[2];
    ctx->ekf.Data.A[15] = ctx->half_gyro_dt[0];
    ctx->ekf.Data.A[18] = ctx->half_gyro_dt[2];
    ctx->ekf.Data.A[19] = ctx->half_gyro_dt[1];
    ctx->ekf.Data.A[20] = -ctx->half_gyro_dt[0];

    memcpy(accel_meas, accel, sizeof(accel_meas));
    if (ctx->acc_lpf_coef > 0.0f) {
        if (ctx->output.initialized == 0U) {
            memcpy(ctx->accel_lpf, accel_meas, sizeof(ctx->accel_lpf));
        }

        alpha = dt / (ctx->acc_lpf_coef + dt);
        for (i = 0U; i < 3U; i++) {
            ctx->accel_lpf[i] += (accel_meas[i] - ctx->accel_lpf[i]) * alpha;
            ctx->output.accel[i] = ctx->accel_lpf[i];
        }
    } else {
        memcpy(ctx->output.accel, accel_meas, sizeof(ctx->output.accel));
    }

    ctx->accel_inv_norm = dm_imu_l1_attitude_inv_sqrt(ctx->output.accel[0] * ctx->output.accel[0] +
                                                       ctx->output.accel[1] * ctx->output.accel[1] +
                                                       ctx->output.accel[2] * ctx->output.accel[2]);

    ctx->ekf.MeasuredVector[0] = ctx->output.accel[0] * ctx->accel_inv_norm;
    ctx->ekf.MeasuredVector[1] = ctx->output.accel[1] * ctx->accel_inv_norm;
    ctx->ekf.MeasuredVector[2] = ctx->output.accel[2] * ctx->accel_inv_norm;

    gyro_norm = 1.0f / ctx->gyro_inv_norm;
    accel_norm = 1.0f / ctx->accel_inv_norm;

    ctx->output.stable_flag = (uint8_t)((gyro_norm < k_stable_gyro_limit) &&
                                        (accel_norm > (k_gravity_accel - k_stable_accel_limit)) &&
                                        (accel_norm < (k_gravity_accel + k_stable_accel_limit)));

    ctx->ekf.ChiSquareTest.TestFlag = (ctx->output.stable_flag != 0U);

    ctx->ekf.Data.Q[0] = ctx->q1 * dt;
    ctx->ekf.Data.Q[7] = ctx->q1 * dt;
    ctx->ekf.Data.Q[14] = ctx->q1 * dt;
    ctx->ekf.Data.Q[21] = ctx->q1 * dt;
    ctx->ekf.Data.Q[28] = ctx->q2 * dt;
    ctx->ekf.Data.Q[35] = ctx->q2 * dt;

    if (ctx->lambda > 0.0f && ctx->lambda < 1.0f) {
        ctx->ekf.Data.P[28] /= ctx->lambda;
        ctx->ekf.Data.P[35] /= ctx->lambda;
    }

    ctx->ekf.Data.R[0] = ctx->r;
    ctx->ekf.Data.R[4] = ctx->r;
    ctx->ekf.Data.R[8] = ctx->r;

    Kalman_Filter_Update(&ctx->ekf);

    ctx->output.quat[0] = ctx->ekf.Output[0];
    ctx->output.quat[1] = ctx->ekf.Output[1];
    ctx->output.quat[2] = ctx->ekf.Output[2];
    ctx->output.quat[3] = ctx->ekf.Output[3];

    ctx->output.gyro_bias[0] = ctx->ekf.Output[4];
    ctx->output.gyro_bias[1] = ctx->ekf.Output[5];
    ctx->output.gyro_bias[2] = 0.0f;

    ctx->output.euler_rad[0] = atan2f(2.0f * (ctx->output.quat[0] * ctx->output.quat[3] + ctx->output.quat[1] * ctx->output.quat[2]),
                                      1.0f - 2.0f * (ctx->output.quat[2] * ctx->output.quat[2] + ctx->output.quat[3] * ctx->output.quat[3]));
    ctx->output.euler_rad[1] = asinf(2.0f * (ctx->output.quat[0] * ctx->output.quat[2]) - 2.0f * (ctx->output.quat[1] * ctx->output.quat[3]));
    ctx->output.euler_rad[2] = atan2f(2.0f * (ctx->output.quat[0] * ctx->output.quat[1] + ctx->output.quat[2] * ctx->output.quat[3]),
                                      1.0f - 2.0f * (ctx->output.quat[1] * ctx->output.quat[1] + ctx->output.quat[2] * ctx->output.quat[2]));

    ctx->output.euler_deg[0] = ctx->output.euler_rad[0] * DEG_PER_RAD;
    ctx->output.euler_deg[1] = ctx->output.euler_rad[1] * DEG_PER_RAD;
    ctx->output.euler_deg[2] = ctx->output.euler_rad[2] * DEG_PER_RAD;

    ctx->output.initialized = 1U;
    return 1U;
}

const dm_imu_l1_attitude_t *dm_imu_l1_attitude_get(uint8_t imu_index)
{
    if (imu_index >= DM_IMU_L1_ATTITUDE_IMU_COUNT) {
        return NULL;
    }

    if (s_ctx[imu_index].active == 0U) {
        return NULL;
    }

    return &s_ctx[imu_index].output;
}

static void dm_imu_l1_attitude_a_update(KalmanFilter_Info_TypeDef *kf)
{
    dm_imu_l1_attitude_ctx_t *ctx = dm_imu_l1_attitude_get_ctx_from_kf(kf);
    float inv_norm;

    if (ctx == NULL) {
        return;
    }

    memset(kf->Data.Cache_Vector[0], 0, kf->sizeof_float * kf->Xhat_Size);
    inv_norm = dm_imu_l1_attitude_inv_sqrt(kf->Data.Xhatminus[0] * kf->Data.Xhatminus[0] +
                                           kf->Data.Xhatminus[1] * kf->Data.Xhatminus[1] +
                                           kf->Data.Xhatminus[2] * kf->Data.Xhatminus[2] +
                                           kf->Data.Xhatminus[3] * kf->Data.Xhatminus[3]);

    kf->Data.Cache_Vector[0][0] = inv_norm;
    kf->Data.Xhatminus[0] *= inv_norm;
    kf->Data.Xhatminus[1] *= inv_norm;
    kf->Data.Xhatminus[2] *= inv_norm;
    kf->Data.Xhatminus[3] *= inv_norm;

    kf->Data.A[4] = 0.5f * kf->Data.Xhatminus[1] * kf->dt;
    kf->Data.A[5] = 0.5f * kf->Data.Xhatminus[2] * kf->dt;
    kf->Data.A[10] = -0.5f * kf->Data.Xhatminus[0] * kf->dt;
    kf->Data.A[11] = 0.5f * kf->Data.Xhatminus[3] * kf->dt;
    kf->Data.A[16] = -0.5f * kf->Data.Xhatminus[3] * kf->dt;
    kf->Data.A[17] = -0.5f * kf->Data.Xhatminus[0] * kf->dt;
    kf->Data.A[22] = 0.5f * kf->Data.Xhatminus[2] * kf->dt;
    kf->Data.A[23] = -0.5f * kf->Data.Xhatminus[1] * kf->dt;

    if (kf->Data.P[28] > 10000.0f) {
        kf->Data.P[28] = 10000.0f;
    } else if (kf->Data.P[28] < -10000.0f) {
        kf->Data.P[28] = -10000.0f;
    }

    if (kf->Data.P[35] > 10000.0f) {
        kf->Data.P[35] = 10000.0f;
    } else if (kf->Data.P[35] < -10000.0f) {
        kf->Data.P[35] = -10000.0f;
    }

    (void)ctx;
}

static void dm_imu_l1_attitude_h_update(KalmanFilter_Info_TypeDef *kf)
{
    memset(kf->Data.H, 0, kf->sizeof_float * kf->Z_Size * kf->Xhat_Size);
    kf->Data.H[0] = -2.0f * kf->Data.Xhatminus[2];
    kf->Data.H[1] = 2.0f * kf->Data.Xhatminus[3];
    kf->Data.H[2] = -2.0f * kf->Data.Xhatminus[0];
    kf->Data.H[3] = 2.0f * kf->Data.Xhatminus[1];
    kf->Data.H[6] = 2.0f * kf->Data.Xhatminus[1];
    kf->Data.H[7] = 2.0f * kf->Data.Xhatminus[0];
    kf->Data.H[8] = 2.0f * kf->Data.Xhatminus[3];
    kf->Data.H[9] = 2.0f * kf->Data.Xhatminus[2];
    kf->Data.H[12] = 2.0f * kf->Data.Xhatminus[0];
    kf->Data.H[13] = -2.0f * kf->Data.Xhatminus[1];
    kf->Data.H[14] = -2.0f * kf->Data.Xhatminus[2];
    kf->Data.H[15] = 2.0f * kf->Data.Xhatminus[3];
}

static uint8_t dm_imu_l1_attitude_chi_sqrt_test(KalmanFilter_Info_TypeDef *kf, dm_imu_l1_attitude_ctx_t *ctx)
{
    kf->Mat.Cache_Matrix[0].numRows = kf->Mat.Cache_Matrix[1].numRows;
    kf->Mat.Cache_Matrix[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Cache_Matrix[1], &kf->Mat.Cache_Vector[1], &kf->Mat.Cache_Matrix[0]);

    kf->Mat.Cache_Vector[0].numRows = 1;
    kf->Mat.Cache_Vector[0].numCols = kf->Mat.Cache_Matrix[1].numRows;
    kf->MatStatus = Matrix_Transpose(&kf->Mat.Cache_Matrix[1], &kf->Mat.Cache_Vector[0]);
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Cache_Vector[0], &kf->Mat.Cache_Matrix[0], &kf->ChiSquareTest.ChiSquare_Matrix);

    if (kf->ChiSquareTest.ChiSquare_Data[0] < 0.5f * kf->ChiSquareTest.ChiSquareTestThresholds) {
        kf->ChiSquareTest.Result = true;
    }

    if (kf->ChiSquareTest.ChiSquare_Data[0] > kf->ChiSquareTest.ChiSquareTestThresholds && kf->ChiSquareTest.Result) {
        if (kf->ChiSquareTest.TestFlag) {
            kf->ChiSquareTest.ChiSquareCnt++;
        } else {
            kf->ChiSquareTest.ChiSquareCnt = 0U;
        }

        if (kf->ChiSquareTest.ChiSquareCnt > 50U) {
            kf->ChiSquareTest.Result = false;
            kf->SkipStep5 = false;
        } else {
            memcpy(kf->Data.Xhat, kf->Data.Xhatminus, kf->sizeof_float * kf->Xhat_Size);
            memcpy(kf->Data.P, kf->Data.Pminus, kf->sizeof_float * kf->Xhat_Size * kf->Xhat_Size);
            kf->SkipStep5 = true;
            return 1U;
        }
    } else {
        if (kf->ChiSquareTest.ChiSquare_Data[0] > 0.1f * kf->ChiSquareTest.ChiSquareTestThresholds && kf->ChiSquareTest.Result) {
            kf->Data.Cache_Vector[0][0] = (kf->ChiSquareTest.ChiSquareTestThresholds - kf->ChiSquareTest.ChiSquare_Data[0]) /
                                          (0.9f * kf->ChiSquareTest.ChiSquareTestThresholds);
        } else {
            kf->Data.Cache_Vector[0][0] = 1.0f;
        }
        kf->ChiSquareTest.ChiSquareCnt = 0U;
        kf->SkipStep5 = false;
    }

    (void)ctx;
    return 0U;
}

static void dm_imu_l1_attitude_xhat_update(KalmanFilter_Info_TypeDef *kf)
{
    dm_imu_l1_attitude_ctx_t *ctx = dm_imu_l1_attitude_get_ctx_from_kf(kf);
    float orientation_cosine[3];
    uint8_t i;
    uint8_t row;
    uint8_t col;

    if (ctx == NULL) {
        return;
    }

    kf->MatStatus = Matrix_Transpose(&kf->Mat.H, &kf->Mat.HT);

    kf->Mat.Cache_Matrix[0].numRows = kf->Mat.H.numRows;
    kf->Mat.Cache_Matrix[0].numCols = kf->Mat.Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.H, &kf->Mat.Pminus, &kf->Mat.Cache_Matrix[0]);

    kf->Mat.Cache_Matrix[1].numRows = kf->Mat.Cache_Matrix[0].numRows;
    kf->Mat.Cache_Matrix[1].numCols = kf->Mat.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Cache_Matrix[0], &kf->Mat.HT, &kf->Mat.Cache_Matrix[1]);

    kf->Mat.K_denominator.numRows = kf->Mat.R.numRows;
    kf->Mat.K_denominator.numCols = kf->Mat.R.numCols;
    kf->MatStatus = Matrix_Add(&kf->Mat.Cache_Matrix[1], &kf->Mat.R, &kf->Mat.K_denominator);
    kf->MatStatus = Matrix_Inverse(&kf->Mat.K_denominator, &kf->Mat.Cache_Matrix[1]);

    kf->Mat.Cache_Vector[0].numRows = kf->Mat.H.numRows;
    kf->Mat.Cache_Vector[0].numCols = 1;
    kf->Data.Cache_Vector[0][0] = 2.0f * (kf->Data.Xhatminus[1] * kf->Data.Xhatminus[3] - kf->Data.Xhatminus[0] * kf->Data.Xhatminus[2]);
    kf->Data.Cache_Vector[0][1] = 2.0f * (kf->Data.Xhatminus[0] * kf->Data.Xhatminus[1] + kf->Data.Xhatminus[2] * kf->Data.Xhatminus[3]);
    kf->Data.Cache_Vector[0][2] = kf->Data.Xhatminus[0] * kf->Data.Xhatminus[0] - kf->Data.Xhatminus[1] * kf->Data.Xhatminus[1] -
                                  kf->Data.Xhatminus[2] * kf->Data.Xhatminus[2] + kf->Data.Xhatminus[3] * kf->Data.Xhatminus[3];

    for (i = 0U; i < 3U; i++) {
        orientation_cosine[i] = acosf(fabsf(kf->Data.Cache_Vector[0][i]));
    }

    kf->Mat.Cache_Vector[1].numRows = kf->Mat.Z.numRows;
    kf->Mat.Cache_Vector[1].numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->Mat.Z, &kf->Mat.Cache_Vector[0], &kf->Mat.Cache_Vector[1]);

    if (dm_imu_l1_attitude_chi_sqrt_test(kf, ctx) != 0U) {
        ctx->output.adaptive_gain_scale = 0.0f;
        return;
    }

    kf->Mat.Cache_Matrix[0].numRows = kf->Mat.Pminus.numRows;
    kf->Mat.Cache_Matrix[0].numCols = kf->Mat.HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Pminus, &kf->Mat.HT, &kf->Mat.Cache_Matrix[0]);
    kf->MatStatus = Matrix_Multiply(&kf->Mat.Cache_Matrix[0], &kf->Mat.Cache_Matrix[1], &kf->Mat.K);

    ctx->output.adaptive_gain_scale = kf->Data.Cache_Vector[0][0];
    if (ctx->output.adaptive_gain_scale <= 0.0f) {
        ctx->output.adaptive_gain_scale = 1.0f;
    }

    for (i = 0U; i < (uint8_t)(kf->Mat.K.numCols * kf->Mat.K.numRows); i++) {
        kf->Data.K[i] *= ctx->output.adaptive_gain_scale;
    }

    for (row = 4U; row < 6U; row++) {
        for (col = 0U; col < 3U; col++) {
            kf->Data.K[row * 3U + col] *= orientation_cosine[row - 4U] / ((float)M_PI * 0.5f);
        }
    }

    kf->Mat.Cache_Vector[0].numRows = kf->Mat.K.numRows;
    kf->Mat.Cache_Vector[0].numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->Mat.K, &kf->Mat.Cache_Vector[1], &kf->Mat.Cache_Vector[0]);

    if (kf->ChiSquareTest.Result) {
        float limit = 1e-2f * kf->dt;

        if (kf->Data.Cache_Vector[0][4] > limit) {
            kf->Data.Cache_Vector[0][4] = limit;
        } else if (kf->Data.Cache_Vector[0][4] < -limit) {
            kf->Data.Cache_Vector[0][4] = -limit;
        }

        if (kf->Data.Cache_Vector[0][5] > limit) {
            kf->Data.Cache_Vector[0][5] = limit;
        } else if (kf->Data.Cache_Vector[0][5] < -limit) {
            kf->Data.Cache_Vector[0][5] = -limit;
        }
    }

    kf->Data.Cache_Vector[0][3] = 0.0f;
    kf->MatStatus = Matrix_Add(&kf->Mat.Xhatminus, &kf->Mat.Cache_Vector[0], &kf->Mat.Xhat);

    if (kf->ChiSquareTest.Result) {
        if (ctx->output.stable_flag) {
            if (ctx->output.error_count < 200U) {
                ctx->output.error_count++;
            }
        } else {
            ctx->output.error_count = 0U;
        }
    }

    if (ctx->output.error_count > 50U) {
        ctx->output.converge_flag = 1U;
    }
}
