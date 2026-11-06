/**
 * @file    gimbal_cali.c
 * @brief   云台校准模块实现 — 从 gimbal_task.c 拆分。
 */

#include "gimbal/gimbal_cali.h"
#include "gimbal/gimbal_task.h"
#include "detect.h"

/* =========================================================================
 * 辅助函数
 * ========================================================================= */

/**
 * @brief 编码器值→相对角度变化转换。
 *
 * @param ecd        当前编码器值。
 * @param offset_ecd 零位偏移编码器值。
 * @return 相对角度，单位 rad。
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t delta = (int32_t)ecd - (int32_t)offset_ecd;
    const int32_t encoder_span = (int32_t)ECD_RANGE + 1;

    while (delta > (encoder_span / 2)) { delta -= encoder_span; }
    while (delta < -(encoder_span / 2)) { delta += encoder_span; }

    return (fp32)delta * MOTOR_ECD_TO_RAD;
}

/* =========================================================================
 * 校准计算
 * ========================================================================= */

void calc_gimbal_axis_cali(uint16_t max_ecd, uint16_t min_ecd,
                           uint16_t *offset, fp32 *max_angle, fp32 *min_angle)
{
    int32_t temp_ecd = 0;
    const int32_t encoder_span = (int32_t)ECD_RANGE + 1;

    if (offset == NULL || max_angle == NULL || min_angle == NULL) {
        return;
    }

    temp_ecd = (int32_t)max_ecd - (int32_t)min_ecd;
    if (temp_ecd < 0) {
        temp_ecd += encoder_span;
    }
    temp_ecd = (int32_t)max_ecd - temp_ecd / 2;
    while (temp_ecd < 0) {
        temp_ecd += encoder_span;
    }
    while (temp_ecd >= encoder_span) {
        temp_ecd -= encoder_span;
    }

    *offset = (uint16_t)temp_ecd;
    *max_angle = motor_ecd_to_angle_change(max_ecd, *offset);
    *min_angle = motor_ecd_to_angle_change(min_ecd, *offset);
}

void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali,
                      uint16_t *yaw_offset, uint16_t *pitch_offset,
                      fp32 *max_yaw, fp32 *min_yaw,
                      fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL ||
        max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL) {
        return;
    }

    calc_gimbal_axis_cali(gimbal_cali->max_yaw_ecd, gimbal_cali->min_yaw_ecd,
                          yaw_offset, max_yaw, min_yaw);
    calc_gimbal_axis_cali(gimbal_cali->max_pitch_ecd, gimbal_cali->min_pitch_ecd,
                          pitch_offset, max_pitch, min_pitch);
}

/* =========================================================================
 * 校准状态机 Hook
 * ========================================================================= */

bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset,
                            fp32 *max_yaw, fp32 *min_yaw,
                            fp32 *max_pitch, fp32 *min_pitch)
{
    extern gimbal_control_t gimbal_control;

    const motor_measure_t *inner_yaw_measure = gimbal_control.gimbal_yaw_motor[INNER_YAW].gimbal_motor_measure;
    const motor_measure_t *outer_yaw_measure = gimbal_control.gimbal_yaw_motor[OUTER_YAW].gimbal_motor_measure;
    const uint8_t inner_yaw_online = (uint8_t)(inner_yaw_measure != NULL &&
                                                inner_yaw_measure->motor_yaw_measure != NULL &&
                                                !toe_is_error(INNER_YAW_GIMBAL_MOTOR_TOE));
    const uint8_t outer_yaw_online = (uint8_t)(outer_yaw_measure != NULL &&
                                                outer_yaw_measure->motor_yaw_measure != NULL &&
                                                !toe_is_error(OUTER_YAW_GIMBAL_MOTOR_TOE));

    if (!yaw_offset || !pitch_offset || !max_yaw || !min_yaw || !max_pitch || !min_pitch) {
        return 0U;
    }

    if (gimbal_control.gimbal_cali.step == 0U) {
        if (!outer_yaw_online && !inner_yaw_online) {
            return 0U;
        }

        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        if (inner_yaw_online && outer_yaw_online) {
            gimbal_control.gimbal_cali.outer_yaw_only = GIMBAL_CALI_DUAL_AXIS_MODE;
        } else if (outer_yaw_online) {
            gimbal_control.gimbal_cali.outer_yaw_only = GIMBAL_CALI_OUTER_YAW_ONLY_MODE;
        } else {
            gimbal_control.gimbal_cali.outer_yaw_only = GIMBAL_CALI_INNER_YAW_ONLY_MODE;
        }

        if (outer_yaw_online) {
            gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_yaw_motor[OUTER_YAW].absolute_angle;
            gimbal_control.gimbal_cali.max_pitch_ecd = outer_yaw_measure->motor_yaw_measure->ecd;
            gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_yaw_motor[OUTER_YAW].absolute_angle;
            gimbal_control.gimbal_cali.min_pitch_ecd = outer_yaw_measure->motor_yaw_measure->ecd;
        }

        if (inner_yaw_online) {
            gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor[INNER_YAW].absolute_angle;
            gimbal_control.gimbal_cali.max_yaw_ecd = inner_yaw_measure->motor_yaw_measure->ecd;
            gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor[INNER_YAW].absolute_angle;
            gimbal_control.gimbal_cali.min_yaw_ecd = inner_yaw_measure->motor_yaw_measure->ecd;
        }
        return 0U;
    }

    if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP) {
        if (gimbal_control.gimbal_cali.outer_yaw_only == GIMBAL_CALI_OUTER_YAW_ONLY_MODE) {
            *yaw_offset = gimbal_control.gimbal_yaw_motor[INNER_YAW].offset_ecd;
            *max_yaw = gimbal_control.gimbal_yaw_motor[INNER_YAW].max_relative_angle;
            *min_yaw = gimbal_control.gimbal_yaw_motor[INNER_YAW].min_relative_angle;
            calc_gimbal_axis_cali(gimbal_control.gimbal_cali.max_pitch_ecd,
                                  gimbal_control.gimbal_cali.min_pitch_ecd,
                                  pitch_offset, max_pitch, min_pitch);
            *max_pitch -= GIMBAL_CALI_REDUNDANT_ANGLE;
            *min_pitch += GIMBAL_CALI_REDUNDANT_ANGLE;
        } else if (gimbal_control.gimbal_cali.outer_yaw_only == GIMBAL_CALI_INNER_YAW_ONLY_MODE) {
            calc_gimbal_axis_cali(gimbal_control.gimbal_cali.max_yaw_ecd,
                                  gimbal_control.gimbal_cali.min_yaw_ecd,
                                  yaw_offset, max_yaw, min_yaw);
            *pitch_offset = gimbal_control.gimbal_yaw_motor[OUTER_YAW].offset_ecd;
            *max_pitch = gimbal_control.gimbal_yaw_motor[OUTER_YAW].max_relative_angle;
            *min_pitch = gimbal_control.gimbal_yaw_motor[OUTER_YAW].min_relative_angle;
            *max_yaw -= GIMBAL_CALI_REDUNDANT_ANGLE;
            *min_yaw += GIMBAL_CALI_REDUNDANT_ANGLE;
        } else {
            calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset,
                             max_yaw, min_yaw, max_pitch, min_pitch);
            *max_yaw -= GIMBAL_CALI_REDUNDANT_ANGLE;
            *min_yaw += GIMBAL_CALI_REDUNDANT_ANGLE;
            *max_pitch -= GIMBAL_CALI_REDUNDANT_ANGLE;
            *min_pitch += GIMBAL_CALI_REDUNDANT_ANGLE;
        }

        set_cali_gimbal_hook(*yaw_offset, *pitch_offset, *max_yaw, *min_yaw, *max_pitch, *min_pitch);
        gimbal_control.gimbal_cali.step = 0U;
        gimbal_control.gimbal_cali.outer_yaw_only = GIMBAL_CALI_DUAL_AXIS_MODE;
        return 1U;
    }

    return 0U;
}

void set_cali_gimbal_hook(uint16_t yaw_offset, uint16_t pitch_offset,
                          fp32 max_yaw, fp32 min_yaw,
                          fp32 max_pitch, fp32 min_pitch)
{
    extern gimbal_control_t gimbal_control;

    gimbal_control.gimbal_yaw_motor[INNER_YAW].offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor[INNER_YAW].max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor[INNER_YAW].min_relative_angle = min_yaw;
    gimbal_control.gimbal_yaw_motor[OUTER_YAW].offset_ecd = pitch_offset;
    gimbal_control.gimbal_yaw_motor[OUTER_YAW].max_relative_angle = max_pitch;
    gimbal_control.gimbal_yaw_motor[OUTER_YAW].min_relative_angle = min_pitch;
}
