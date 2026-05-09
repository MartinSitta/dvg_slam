#ifndef VOXELHASHMAP_H
#define VOXELHASHMAP_H

#ifdef __cplusplus
extern "C"{
#endif

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "FibonacciHash.h"
#include "HashUtils.h"
#include "PointSlot.h"
typedef struct VoxelHashMap_t 
{
    PointSlot_t* slots;
    uint32_t hash_seed;
    uint64_t capacity;
    uint64_t occupied_slot_count;
    uint64_t tombstome_count;
    uint8_t max_probe_chain_len;
    float load_factor_threshold;
    /* data */
}VoxelHashMap_t;

VoxelHashMap_t* voxel_hash_map_init(uint64_t intial_capacity, uint8_t probe_chain_limit, float max_load_factor);
void voxel_hash_map_free(VoxelHashMap_t* hashmap);

PointSlot_t* voxel_hash_map_insert(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z);
PointSlot_t* voxel_hash_map_lookup(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z);
bool voxel_hash_map_remove(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z);

#ifdef __cplusplus
}
#endif
#endif