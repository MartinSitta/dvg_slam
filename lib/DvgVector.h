#ifndef DVGVECTOR_H
#define DVGVECTOR_H

#ifdef __cplusplus
extern "C"{
#endif
#include <math.h>

typedef struct{
    double x;
    double y;
    double z;
} DvgVector_t;

DvgVector_t dvg_vector_normalize(DvgVector_t vector);
#ifdef __cplusplus
}
#endif

#endif