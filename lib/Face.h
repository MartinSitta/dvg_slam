#ifndef FACE_H
#define FACE_H

#ifdef __cplusplus
extern "C"{
#endif

#include "GraphDefines.h"
typedef enum{
    UP,
    DOWN,
    LEFT,
    RIGHT
} orientation_t;


typedef struct{
    uint32_t vertices[4];
}Face_t;


#ifdef __cplusplus
}
#endif

#endif
