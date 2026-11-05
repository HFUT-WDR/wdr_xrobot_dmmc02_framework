#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
typedef struct {uint16_t numRows,numCols;float *pData;} arm_matrix_instance_f32;
typedef struct {uint16_t numRows,numCols;double *pData;} arm_matrix_instance_f64;
typedef int arm_status;
enum {ARM_MATH_SUCCESS=0,ARM_MATH_LENGTH_ERROR=-1,ARM_MATH_SIZE_MISMATCH=-3,ARM_MATH_SINGULAR=-5};
static inline void arm_mat_init_f32(arm_matrix_instance_f32 *m,uint16_t r,uint16_t c,float *p){m->numRows=r;m->numCols=c;m->pData=p;}
static inline int arm_mat_add_f32(const arm_matrix_instance_f32 *a,const arm_matrix_instance_f32 *b,arm_matrix_instance_f32 *o){if(a->numRows!=b->numRows||a->numCols!=b->numCols)return -3;for(unsigned i=0;i<a->numRows*a->numCols;++i)o->pData[i]=a->pData[i]+b->pData[i];return 0;}
static inline int arm_mat_sub_f32(const arm_matrix_instance_f32 *a,const arm_matrix_instance_f32 *b,arm_matrix_instance_f32 *o){if(a->numRows!=b->numRows||a->numCols!=b->numCols)return -3;for(unsigned i=0;i<a->numRows*a->numCols;++i)o->pData[i]=a->pData[i]-b->pData[i];return 0;}
static inline int arm_mat_trans_f32(const arm_matrix_instance_f32 *a,arm_matrix_instance_f32 *o){for(unsigned r=0;r<a->numRows;++r)for(unsigned c=0;c<a->numCols;++c)o->pData[c*a->numRows+r]=a->pData[r*a->numCols+c];return 0;}
static inline int arm_mat_mult_f32(const arm_matrix_instance_f32 *a,const arm_matrix_instance_f32 *b,arm_matrix_instance_f32 *o){if(a->numCols!=b->numRows)return -3;for(unsigned r=0;r<a->numRows;++r)for(unsigned c=0;c<b->numCols;++c){float v=0;for(unsigned k=0;k<a->numCols;++k)v+=a->pData[r*a->numCols+k]*b->pData[k*b->numCols+c];o->pData[r*b->numCols+c]=v;}return 0;}
/* Pc scalar Gauss-Jordan backend for both oracle and migrated code, not a CMSIS timing/rounding emulator. */
static inline int arm_mat_inverse_f32(const arm_matrix_instance_f32 *a,arm_matrix_instance_f32 *o){
 unsigned n=a->numRows;if(n!=a->numCols)return -3;
 memset(o->pData,0,n*n*sizeof(float));for(unsigned r=0;r<n;++r)o->pData[r*n+r]=1;
 for(unsigned c=0;c<n;++c){unsigned p=c;for(unsigned r=c+1;r<n;++r)if(fabsf(a->pData[r*n+c])>fabsf(a->pData[p*n+c]))p=r;
  if(a->pData[p*n+c]==0)return -5;
  for(unsigned k=0;k<n;++k){float t=a->pData[c*n+k];a->pData[c*n+k]=a->pData[p*n+k];a->pData[p*n+k]=t;t=o->pData[c*n+k];o->pData[c*n+k]=o->pData[p*n+k];o->pData[p*n+k]=t;}
  float d=a->pData[c*n+c];for(unsigned k=0;k<n;++k){a->pData[c*n+k]/=d;o->pData[c*n+k]/=d;}
  for(unsigned r=0;r<n;++r)if(r!=c){float f=a->pData[r*n+c];for(unsigned k=0;k<n;++k){a->pData[r*n+k]-=f*a->pData[c*n+k];o->pData[r*n+k]-=f*o->pData[c*n+k];}}
 }return 0;
}
