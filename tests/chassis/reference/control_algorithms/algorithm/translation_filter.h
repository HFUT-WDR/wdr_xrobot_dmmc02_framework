#pragma once
#ifdef __cplusplus
extern "C" {
#endif
typedef float fp32;
typedef struct { fp32 input, out, num[1], frame_period; } first_order_filter_type_t;
void first_order_filter_init(first_order_filter_type_t*, fp32, const fp32[1]);
void first_order_filter_cali(first_order_filter_type_t*, fp32);
#ifdef __cplusplus
}
#endif
