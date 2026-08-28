#ifndef VOXELPRIORITYQUEUE_H
#define VOXELPRIORITYQUEUE_H

#ifdef __cplusplus
extern "C"{
#endif
#include "PointSlot.h"
#include "VoxelHashMap.h"
#include <stddef.h>
#include <assert.h>
#include <stdlib.h>
typedef struct VoxelPriorityQueue_t{
    Point_t* array;
    int64_t capacity;
    int64_t current_element;
}VoxelPriorityQueue_t;
typedef struct DequeueRetObject_t{
    Point_t point;
    bool valid;
} DequeueRetObject_t;
VoxelPriorityQueue_t* voxel_priority_queue_init(int64_t capacity);
void voxel_priority_queue_free(VoxelPriorityQueue_t* queue);
void voxel_priority_queue_enqueue(VoxelPriorityQueue_t*, Point_t point, VoxelHashMap_t* nodes);
DequeueRetObject_t voxel_priority_queue_dequeue(VoxelPriorityQueue_t*, VoxelHashMap_t* nodes);
Point_t* voxel_priority_queue_peek(VoxelPriorityQueue_t*);


#ifdef __cplusplus
};
#endif



#endif