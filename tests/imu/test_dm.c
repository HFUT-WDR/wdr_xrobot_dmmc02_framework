#include "dm_imu_l1_attitude.c"
static dm_imu_l1_attitude_ctx_t attitude_contexts[2];
static void actual_reset(unsigned i) { if(i<2) dm_imu_l1_attitude_reset(&attitude_contexts[i]); }
static unsigned actual_update(unsigned i,const float *g,const float *a,float dt) { return i<2 ? dm_imu_l1_attitude_update(&attitude_contexts[i],g,a,dt) : 0; }
static const dm_imu_l1_attitude_t *actual_get(unsigned i) { return i<2 && attitude_contexts[i].active ? &attitude_contexts[i].output : NULL; }
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
void reference_reset(uint8_t index);
uint8_t reference_update(uint8_t index,const float gyro[3],const float accel[3],float dt);
const dm_imu_l1_attitude_t *reference_get(uint8_t index);
static void *allocations[256];static unsigned allocation_count;
void *tracked_malloc(size_t size){assert(allocation_count<256);void *p=malloc(size);assert(p);allocations[allocation_count++]=p;return p;}
static void compare(unsigned index){
 const dm_imu_l1_attitude_t *a=actual_get(index),*b=reference_get(index);assert(a&&b);
 assert(a->initialized==b->initialized&&a->stable_flag==b->stable_flag&&a->converge_flag==b->converge_flag&&a->error_count==b->error_count);
 const float *av[]={a->quat,a->euler_rad,a->euler_deg,a->gyro_bias,a->gyro,a->accel,&a->adaptive_gain_scale};
 const float *bv[]={b->quat,b->euler_rad,b->euler_deg,b->gyro_bias,b->gyro,b->accel,&b->adaptive_gain_scale};
 const unsigned lengths[]={4,3,3,3,3,3,1};
 for(unsigned j=0;j<7;++j)for(unsigned k=0;k<lengths[j];++k){assert(isfinite(av[j][k]));assert(av[j][k]==bv[j][k]);}
}
int main(void){
 assert(actual_get(0)==NULL&&actual_get(2)==NULL);
 // Independent scalar check of the repaired chi-square expression: 2*1²+3*2²+4*3²=50.
 actual_reset(0);
 kalman_filter_info_t *kf=&attitude_contexts[0].ekf;
 kf->mat.cache_matrix[1].numRows=kf->mat.cache_matrix[1].numCols=3;
 kf->data.cache_matrix[1][0]=2;kf->data.cache_matrix[1][4]=3;kf->data.cache_matrix[1][8]=4;
 kf->mat.cache_vector[1].numRows=3;kf->mat.cache_vector[1].numCols=1;
 kf->data.cache_vector[1][0]=1;kf->data.cache_vector[1][1]=2;kf->data.cache_vector[1][2]=3;
 kf->chi_square_test.chi_square_test_thresholds=200;
 assert(dm_imu_l1_attitude_chi_sqrt_test(kf,&attitude_contexts[0])==0);
 assert(kf->chi_square_test.chi_square_data[0]==50);
 actual_reset(0);
 float g[3]={0,0,0},a[3]={0,0,9.80665f};
 assert(!actual_update(2,g,a,.001f));assert(!actual_update(0,NULL,a,.001f));assert(!actual_update(0,g,a,0));
 for(unsigned i=0;i<4000;++i){
  g[0]=i<1000?0:.2f*sinf(i*.01f);g[1]=i<1000?0:.1f*cosf(i*.02f);g[2]=i<1000?0:.4f;
  a[0]=i<2000?0:1.2f*sinf(i*.03f);a[1]=i<2000?0:.3f*cosf(i*.02f);
  for(unsigned index=0;index<2;++index){float gyro[3]={g[0],g[1],index?-.2f:g[2]};
   assert(reference_update(index,gyro,a,.001f));assert(actual_update(index,gyro,a,.001f));compare(index);
  }
 }
 const dm_imu_l1_attitude_t saved=*actual_get(1);unsigned count=allocation_count;
 for(unsigned i=0;i<50;++i)actual_reset(0);
 assert(allocation_count==count&&memcmp(&saved,actual_get(1),sizeof(saved))==0);
 reference_reset(0);g[0]=g[1]=g[2]=0;a[0]=a[1]=0;
 for(unsigned i=0;i<100;++i){assert(reference_update(0,g,a,.001f));assert(actual_update(0,g,a,.001f));compare(0);}
 const dm_imu_l1_attitude_t *out=actual_get(0);assert(out->stable_flag&&fabsf(out->euler_rad[0])<.001f&&fabsf(out->euler_rad[1])<.001f&&fabsf(out->euler_rad[2])<.001f);
 for(unsigned i=0;i<allocation_count;++i)free(allocations[i]);
 puts("DM fusion: 8100 source comparisons, independent instances, stationary axes and allocation-free reset passed");
}
