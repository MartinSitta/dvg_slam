#ifndef POINTSLOT_H
#define POINTSLOT_H

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>

typedef struct Point_t{
int64_t x;
int64_t y;
int64_t z;
}Point_t;

typedef enum{
    SLOT_EMPTY = 1,
    SLOT_OCCUPIED = 2,
    SLOT_TOMBSTONE = 3,
}SlotState_t;
typedef struct PointSlot_t PointSlot_t;
typedef struct PointSlot_t{
    Point_t key;
    Point_t prev_key;
    uint64_t raw_hash; //NOTE: LITERALLY THE RAW HASH. NOT THE CAPACITY AJUSTED ONE!
    SlotState_t state;
    float astar_heuristic;
    float traveled_dist;
    bool visited;
    bool has_prev;
}PointSlot_t;

#ifdef __cplusplus
}
#endif
#endif