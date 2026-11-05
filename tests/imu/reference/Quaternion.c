#include <math.h>
#include <string.h>

#include "Quaternion.h"
#include "Kalman_Filter.h"
#include "arm_math.h"

static float Fast_InverseSqrt(float x);
static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *KF);
static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *KF);
static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *KF);
static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *KF);

Quaternion_Info_Typedef Quaternion_Info;

static const float QUATERNION_STABLE_GYRO_LIMIT = 0.3f;
static const float QUATERNION_STABLE_ACCEL_LIMIT = 0.5f;
static const float QUATERNION_CHI_THRESHOLD = 1e-8f;
static const float QUATERNION_DEFAULT_LAMBDA = 0.9996f;
static const float QUATERNION_DEFAULT_ACCLPF = 0.0f;

void QuaternionEKF_Init(Quaternion_Info_Typedef *Quat, float Process_Noise1, float Process_Noise2, float Measure_Noise,
                        float lambda, float accLPFcoef, float *QuaternionEKF_A_Data, float *QuaternionEKF_P_Data)
{
    Quat->Q1 = Process_Noise1;
    Quat->Q2 = Process_Noise2;
    Quat->R = Measure_Noise;
    Quat->lambda = (lambda > 1.0f) ? 1.0f : ((lambda <= 0.0f) ? QUATERNION_DEFAULT_LAMBDA : lambda);
    Quat->accLPFcoef = (accLPFcoef < 0.0f) ? QUATERNION_DEFAULT_ACCLPF : accLPFcoef;
    Quat->QuaternionEKF_A_Data = QuaternionEKF_A_Data;
    Quat->QuaternionEKF_P_Data = QuaternionEKF_P_Data;
    Quat->StableFlag = false;
    Quat->ConvergeFlag = false;
    Quat->ErrorCount = 0U;
    Quat->AdaptiveGainScale = 1.0f;
    memset(Quat->Accel, 0, sizeof(Quat->Accel));
    memset(Quat->Gyro, 0, sizeof(Quat->Gyro));
    memset(Quat->accel_lpf, 0, sizeof(Quat->accel_lpf));
    memset(Quat->EulerAngle, 0, sizeof(Quat->EulerAngle));

    Kalman_Filter_Init(&Quat->QuaternionEKF, 6, 0, 3);
    Quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
    Quat->QuaternionEKF.ChiSquareTest.Result = false;
    Quat->QuaternionEKF.ChiSquareTest.ChiSquareTestThresholds = QUATERNION_CHI_THRESHOLD;
    Quat->QuaternionEKF.ChiSquareTest.ChiSquareCnt = 0;

    Quat->QuaternionEKF.Data.Xhat[0] = 1.f;
    Quat->QuaternionEKF.Data.Xhat[1] = 0.f;
    Quat->QuaternionEKF.Data.Xhat[2] = 0.f;
    Quat->QuaternionEKF.Data.Xhat[3] = 0.f;
    Quat->QuaternionEKF.Data.Xhat[4] = 0.f;
    Quat->QuaternionEKF.Data.Xhat[5] = 0.f;

    Quat->QuaternionEKF.User_Function1 = QuaternionEKF_A_Update;
    Quat->QuaternionEKF.User_Function2 = QuaternionEKF_H_Update;
    Quat->QuaternionEKF.User_Function3 = QuaternionEKF_xhat_Update;

    Quat->QuaternionEKF.SkipStep3 = true;
    Quat->QuaternionEKF.SkipStep4 = true;

    memcpy(Quat->QuaternionEKF.Data.A, Quat->QuaternionEKF_A_Data, Quat->QuaternionEKF.sizeof_float * 36U);
    memcpy(Quat->QuaternionEKF.Data.P, Quat->QuaternionEKF_P_Data, Quat->QuaternionEKF.sizeof_float * 36U);
}

void QuaternionEKF_Update(Quaternion_Info_Typedef *Quat, float Gyro[3], float Accel[3], float dt)
{
    float accel_meas[3];

    Quat->QuaternionEKF.dt = dt;

    Quat->Gyro[0] = Gyro[0] - Quat->Deviate[0];
    Quat->Gyro[1] = Gyro[1] - Quat->Deviate[1];
    Quat->Gyro[2] = Gyro[2] - Quat->Deviate[2];

    Quat->GyroInvNorm = Fast_InverseSqrt(Quat->Gyro[0] * Quat->Gyro[0] + Quat->Gyro[1] * Quat->Gyro[1] + Quat->Gyro[2] * Quat->Gyro[2]);
    Quat->Half_Gyro_dt[0] = 0.5f * Quat->Gyro[0] * dt;
    Quat->Half_Gyro_dt[1] = 0.5f * Quat->Gyro[1] * dt;
    Quat->Half_Gyro_dt[2] = 0.5f * Quat->Gyro[2] * dt;

    memcpy(Quat->QuaternionEKF.Data.A, Quat->QuaternionEKF_A_Data, Quat->QuaternionEKF.sizeof_float * 36U);

    Quat->QuaternionEKF.Data.A[1] = -Quat->Half_Gyro_dt[0];
    Quat->QuaternionEKF.Data.A[2] = -Quat->Half_Gyro_dt[1];
    Quat->QuaternionEKF.Data.A[3] = -Quat->Half_Gyro_dt[2];
    Quat->QuaternionEKF.Data.A[6] = Quat->Half_Gyro_dt[0];
    Quat->QuaternionEKF.Data.A[8] = Quat->Half_Gyro_dt[2];
    Quat->QuaternionEKF.Data.A[9] = -Quat->Half_Gyro_dt[1];
    Quat->QuaternionEKF.Data.A[12] = Quat->Half_Gyro_dt[1];
    Quat->QuaternionEKF.Data.A[13] = -Quat->Half_Gyro_dt[2];
    Quat->QuaternionEKF.Data.A[15] = Quat->Half_Gyro_dt[0];
    Quat->QuaternionEKF.Data.A[18] = Quat->Half_Gyro_dt[2];
    Quat->QuaternionEKF.Data.A[19] = Quat->Half_Gyro_dt[1];
    Quat->QuaternionEKF.Data.A[20] = -Quat->Half_Gyro_dt[0];

    memcpy(accel_meas, Accel, sizeof(accel_meas));
    if (Quat->accLPFcoef > 0.0f) {
        if (!Quat->Init) {
            memcpy(Quat->accel_lpf, accel_meas, sizeof(Quat->accel_lpf));
        }
        const float alpha = dt / (Quat->accLPFcoef + dt);
        for (uint8_t i = 0; i < 3; i++) {
            Quat->accel_lpf[i] = Quat->accel_lpf[i] + (accel_meas[i] - Quat->accel_lpf[i]) * alpha;
            Quat->Accel[i] = Quat->accel_lpf[i];
        }
    } else {
        memcpy(Quat->Accel, accel_meas, sizeof(Quat->Accel));
    }

    Quat->AccelInvNorm = Fast_InverseSqrt(Quat->Accel[0] * Quat->Accel[0] + Quat->Accel[1] * Quat->Accel[1] + Quat->Accel[2] * Quat->Accel[2]);
    Quat->QuaternionEKF.MeasuredVector[0] = Quat->Accel[0] * Quat->AccelInvNorm;
    Quat->QuaternionEKF.MeasuredVector[1] = Quat->Accel[1] * Quat->AccelInvNorm;
    Quat->QuaternionEKF.MeasuredVector[2] = Quat->Accel[2] * Quat->AccelInvNorm;

    const float gyro_norm = 1.0f / Quat->GyroInvNorm;
    const float accel_norm = 1.0f / Quat->AccelInvNorm;
    Quat->StableFlag = (gyro_norm < QUATERNION_STABLE_GYRO_LIMIT &&
                        accel_norm > (GravityAccel - QUATERNION_STABLE_ACCEL_LIMIT) &&
                        accel_norm < (GravityAccel + QUATERNION_STABLE_ACCEL_LIMIT));

    Quat->QuaternionEKF.ChiSquareTest.TestFlag = Quat->StableFlag;
    Quat->QuaternionEKF.Data.Q[0] = Quat->Q1 * dt;
    Quat->QuaternionEKF.Data.Q[7] = Quat->Q1 * dt;
    Quat->QuaternionEKF.Data.Q[14] = Quat->Q1 * dt;
    Quat->QuaternionEKF.Data.Q[21] = Quat->Q1 * dt;
    Quat->QuaternionEKF.Data.Q[28] = Quat->Q2 * dt;
    Quat->QuaternionEKF.Data.Q[35] = Quat->Q2 * dt;
    if (Quat->lambda > 0.0f && Quat->lambda < 1.0f) {
        Quat->QuaternionEKF.Data.P[28] /= Quat->lambda;
        Quat->QuaternionEKF.Data.P[35] /= Quat->lambda;
    }
    Quat->QuaternionEKF.Data.R[0] = Quat->R;
    Quat->QuaternionEKF.Data.R[4] = Quat->R;
    Quat->QuaternionEKF.Data.R[8] = Quat->R;

    Kalman_Filter_Update(&Quat->QuaternionEKF);

    Quat->quat[0] = Quat->QuaternionEKF.Output[0];
    Quat->quat[1] = Quat->QuaternionEKF.Output[1];
    Quat->quat[2] = Quat->QuaternionEKF.Output[2];
    Quat->quat[3] = Quat->QuaternionEKF.Output[3];
    Quat->Deviate[0] = Quat->QuaternionEKF.Output[4];
    Quat->Deviate[1] = Quat->QuaternionEKF.Output[5];
    Quat->Deviate[2] = 0.f;

    Quat->EulerAngle[0] = atan2f(2.f * (Quat->quat[0] * Quat->quat[3] + Quat->quat[1] * Quat->quat[2]), 1.f - 2.f * (Quat->quat[2] * Quat->quat[2] + Quat->quat[3] * Quat->quat[3]));
    Quat->EulerAngle[1] = asinf(2.f * (Quat->quat[0] * Quat->quat[2]) - 2.f * (Quat->quat[1] * Quat->quat[3]));
    Quat->EulerAngle[2] = atan2f(2.f * (Quat->quat[0] * Quat->quat[1] + Quat->quat[2] * Quat->quat[3]), 1.f - 2.f * (Quat->quat[1] * Quat->quat[1] + Quat->quat[2] * Quat->quat[2]));
    Quat->Init = true;
}

static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *KF)
{
    memset(KF->Data.Cache_Vector[0], 0, KF->sizeof_float * KF->Xhat_Size);
    KF->Data.Cache_Vector[0][0] = Fast_InverseSqrt(KF->Data.Xhatminus[0] * KF->Data.Xhatminus[0] +
                                                   KF->Data.Xhatminus[1] * KF->Data.Xhatminus[1] +
                                                   KF->Data.Xhatminus[2] * KF->Data.Xhatminus[2] +
                                                   KF->Data.Xhatminus[3] * KF->Data.Xhatminus[3]);
    KF->Data.Xhatminus[0] *= KF->Data.Cache_Vector[0][0];
    KF->Data.Xhatminus[1] *= KF->Data.Cache_Vector[0][0];
    KF->Data.Xhatminus[2] *= KF->Data.Cache_Vector[0][0];
    KF->Data.Xhatminus[3] *= KF->Data.Cache_Vector[0][0];

    KF->Data.A[4] = 0.5f * KF->Data.Xhatminus[1] * KF->dt;
    KF->Data.A[5] = 0.5f * KF->Data.Xhatminus[2] * KF->dt;
    KF->Data.A[10] = -0.5f * KF->Data.Xhatminus[0] * KF->dt;
    KF->Data.A[11] = 0.5f * KF->Data.Xhatminus[3] * KF->dt;
    KF->Data.A[16] = -0.5f * KF->Data.Xhatminus[3] * KF->dt;
    KF->Data.A[17] = -0.5f * KF->Data.Xhatminus[0] * KF->dt;
    KF->Data.A[22] = 0.5f * KF->Data.Xhatminus[2] * KF->dt;
    KF->Data.A[23] = -0.5f * KF->Data.Xhatminus[1] * KF->dt;

    VAL_LIMIT(KF->Data.P[28], -10000, 10000);
    VAL_LIMIT(KF->Data.P[35], -10000, 10000);
}

static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *KF)
{
    memset(KF->Data.H, 0, KF->sizeof_float * KF->Z_Size * KF->Xhat_Size);
    KF->Data.H[0] = -2.f * KF->Data.Xhatminus[2];
    KF->Data.H[1] = 2.f * KF->Data.Xhatminus[3];
    KF->Data.H[2] = -2.f * KF->Data.Xhatminus[0];
    KF->Data.H[3] = 2.f * KF->Data.Xhatminus[1];
    KF->Data.H[6] = 2.f * KF->Data.Xhatminus[1];
    KF->Data.H[7] = 2.f * KF->Data.Xhatminus[0];
    KF->Data.H[8] = 2.f * KF->Data.Xhatminus[3];
    KF->Data.H[9] = 2.f * KF->Data.Xhatminus[2];
    KF->Data.H[12] = 2.f * KF->Data.Xhatminus[0];
    KF->Data.H[13] = -2.f * KF->Data.Xhatminus[1];
    KF->Data.H[14] = -2.f * KF->Data.Xhatminus[2];
    KF->Data.H[15] = 2.f * KF->Data.Xhatminus[3];
}

static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *KF)
{
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Cache_Matrix[1].numRows;
    KF->Mat.Cache_Matrix[0].numCols = 1;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Matrix[0]);
    KF->Mat.Cache_Vector[0].numRows = 1;
    KF->Mat.Cache_Vector[0].numCols = KF->Mat.Cache_Matrix[1].numRows;
    KF->MatStatus = Matrix_Transpose(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[0]);
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Matrix[0], &KF->ChiSquareTest.ChiSquare_Matrix);

    if (KF->ChiSquareTest.ChiSquare_Data[0] < 0.5f * KF->ChiSquareTest.ChiSquareTestThresholds) {
        KF->ChiSquareTest.Result = true;
    }

    if (KF->ChiSquareTest.ChiSquare_Data[0] > KF->ChiSquareTest.ChiSquareTestThresholds && KF->ChiSquareTest.Result) {
        if (KF->ChiSquareTest.TestFlag) {
            KF->ChiSquareTest.ChiSquareCnt++;
        } else {
            KF->ChiSquareTest.ChiSquareCnt = 0;
        }

        if (KF->ChiSquareTest.ChiSquareCnt > 50) {
            KF->ChiSquareTest.Result = 0;
            KF->SkipStep5 = false;
        } else {
            memcpy(KF->Data.Xhat, KF->Data.Xhatminus, KF->sizeof_float * KF->Xhat_Size);
            memcpy(KF->Data.P, KF->Data.Pminus, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);
            KF->SkipStep5 = true;
            return true;
        }
    } else {
        if (KF->ChiSquareTest.ChiSquare_Data[0] > 0.1f * KF->ChiSquareTest.ChiSquareTestThresholds && KF->ChiSquareTest.Result) {
            KF->Data.Cache_Vector[0][0] = (KF->ChiSquareTest.ChiSquareTestThresholds - KF->ChiSquareTest.ChiSquare_Data[0]) / (0.9f * KF->ChiSquareTest.ChiSquareTestThresholds);
        } else {
            KF->Data.Cache_Vector[0][0] = 1.f;
        }
        KF->ChiSquareTest.ChiSquareCnt = 0;
        KF->SkipStep5 = false;
    }

    return false;
}

static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *KF)
{
    KF->MatStatus = Matrix_Transpose(&KF->Mat.H, &KF->Mat.HT);
    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.H.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.Pminus.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[0]);
    KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
    KF->Mat.Cache_Matrix[1].numCols = KF->Mat.HT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.HT, &KF->Mat.Cache_Matrix[1]);
    KF->Mat.K_denominator.numRows = KF->Mat.R.numRows;
    KF->Mat.K_denominator.numCols = KF->Mat.R.numCols;
    KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[1], &KF->Mat.R, &KF->Mat.K_denominator);
    KF->MatStatus = Matrix_Inverse(&KF->Mat.K_denominator, &KF->Mat.Cache_Matrix[1]);

    KF->Mat.Cache_Vector[0].numRows = KF->Mat.H.numRows;
    KF->Mat.Cache_Vector[0].numCols = 1;
    KF->Data.Cache_Vector[0][0] = 2.f * (KF->Data.Xhatminus[1] * KF->Data.Xhatminus[3] - KF->Data.Xhatminus[0] * KF->Data.Xhatminus[2]);
    KF->Data.Cache_Vector[0][1] = 2.f * (KF->Data.Xhatminus[0] * KF->Data.Xhatminus[1] + KF->Data.Xhatminus[2] * KF->Data.Xhatminus[3]);
    KF->Data.Cache_Vector[0][2] = KF->Data.Xhatminus[0] * KF->Data.Xhatminus[0] - KF->Data.Xhatminus[1] * KF->Data.Xhatminus[1] - KF->Data.Xhatminus[2] * KF->Data.Xhatminus[2] + KF->Data.Xhatminus[3] * KF->Data.Xhatminus[3];

    float OrientationCosine[3];
    for (uint8_t i = 0; i < 3; i++) {
        OrientationCosine[i] = acosf(fabsf(KF->Data.Cache_Vector[0][i]));
    }

    KF->Mat.Cache_Vector[1].numRows = KF->Mat.Z.numRows;
    KF->Mat.Cache_Vector[1].numCols = 1;
    KF->MatStatus = Matrix_Subtract(&KF->Mat.Z, &KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1]);

    if (QuaternionEKF_ChiSqrtTest(KF) == true) {
        Quaternion_Info.AdaptiveGainScale = 0.0f;
        return;
    }

    KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
    KF->Mat.Cache_Matrix[0].numCols = KF->Mat.HT.numCols;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.HT, &KF->Mat.Cache_Matrix[0]);
    KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Cache_Matrix[1], &KF->Mat.K);

    Quaternion_Info.AdaptiveGainScale = KF->Data.Cache_Vector[0][0];
    if (Quaternion_Info.AdaptiveGainScale <= 0.0f) {
        Quaternion_Info.AdaptiveGainScale = 1.0f;
    }

    for (uint8_t i = 0; i < KF->Mat.K.numCols * KF->Mat.K.numRows; i++) {
        KF->Data.K[i] *= Quaternion_Info.AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            KF->Data.K[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f;
        }
    }

    KF->Mat.Cache_Vector[0].numRows = KF->Mat.K.numRows;
    KF->Mat.Cache_Vector[0].numCols = 1;
    KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Vector[0]);

    if (KF->ChiSquareTest.Result) {
        VAL_LIMIT(KF->Data.Cache_Vector[0][4], -1e-2f * KF->dt, 1e-2f * KF->dt);
        VAL_LIMIT(KF->Data.Cache_Vector[0][5], -1e-2f * KF->dt, 1e-2f * KF->dt);
    }
    KF->Data.Cache_Vector[0][3] = 0;
    KF->MatStatus = Matrix_Add(&KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0], &KF->Mat.Xhat);

    if (KF->ChiSquareTest.Result) {
        if (Quaternion_Info.StableFlag) {
            if (Quaternion_Info.ErrorCount < 200U) {
                Quaternion_Info.ErrorCount++;
            }
        } else {
            Quaternion_Info.ErrorCount = 0U;
        }
    }

    if (Quaternion_Info.ErrorCount > 50U) {
        Quaternion_Info.ConvergeFlag = true;
    }
}

static float Fast_InverseSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
