#include "quaternion.c"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
extern quaternion_info_t reference_info;
void reference_init(quaternion_info_t *,float,float,float,float,float,float *,float *);
void reference_update(quaternion_info_t *,float *,float *,float);
static void *allocations[256];
static unsigned allocation_count;
void *tracked_malloc(size_t size) {void *p=malloc(size);assert(p && allocation_count<256);allocations[allocation_count++]=p;return p;}
int main(void)
{
 float a[36]={0},p[36]={0};
 for(unsigned i=0;i<6;++i){a[i*6+i]=1;for(unsigned j=0;j<6;++j)p[i*6+j]=i==j?(i<4?100000.f:100.f):.1f;}
 quaternion_info_t actual={0},other={0};
 quaternion_ekf_init(&actual,10,.001f,10000000,.9996f,0,a,p);
 quaternion_ekf_init(&other,10,.001f,10000000,.9996f,.005f,a,p);
 reference_init(&reference_info,10,.001f,10000000,.9996f,0,a,p);
 // 独立检验修正后的residual^T S^-1 residual = 50。
 kalman_filter_info_t *kf=&other.quaternion_ekf;
 kf->mat.cache_matrix[1].numRows=kf->mat.cache_matrix[1].numCols=3;
 kf->data.cache_matrix[1][0]=2;kf->data.cache_matrix[1][4]=3;kf->data.cache_matrix[1][8]=4;
 kf->mat.cache_vector[1].numRows=3;kf->mat.cache_vector[1].numCols=1;
 kf->data.cache_vector[1][0]=1;kf->data.cache_vector[1][1]=2;kf->data.cache_vector[1][2]=3;
 kf->chi_square_test.chi_square_test_thresholds=200;
 assert(!quaternion_ekf_chi_sqrt_test(kf));assert(kf->chi_square_test.chi_square_data[0]==50);
 for(unsigned step=0;step<5000;++step){
  float gyro[3]={step<1000?0:.2f*sinf(step*.01f),step<1000?0:.1f*cosf(step*.02f),step<1000?0:.3f};
  float accel[3]={step<2000?0:.5f*sinf(step*.03f),0,9.80665f};
  quaternion_ekf_update(&actual,gyro,accel,.001f);
  reference_update(&reference_info,gyro,accel,.001f);
  assert(actual.stable_flag==reference_info.stable_flag && actual.converge_flag==reference_info.converge_flag && actual.error_count==reference_info.error_count);
  assert(actual.adaptive_gain_scale==reference_info.adaptive_gain_scale);
  for(unsigned i=0;i<4;++i){assert(isfinite(actual.quat[i]));assert(actual.quat[i]==reference_info.quat[i]);}
  for(unsigned i=0;i<3;++i){assert(actual.euler_angle[i]==reference_info.euler_angle[i]);assert(actual.deviate[i]==reference_info.deviate[i]);}
  const uint8_t errors=actual.error_count;const float gain=actual.adaptive_gain_scale;
  float g2[3]={0,0,-.7f};quaternion_ekf_update(&other,g2,accel,.001f);
  assert(actual.error_count==errors && actual.adaptive_gain_scale==gain);
 }
 for(unsigned i=0;i<allocation_count;++i)free(allocations[i]);
 puts("Board EKF: 5000 frozen-source comparisons, residual scalar check and independent instance diagnostics passed");
}
