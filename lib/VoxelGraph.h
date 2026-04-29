#ifndef VOXELGRAPH_H
#define VOXELGRAPH_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Chunk.h"
#include "AltChunk.h"
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include "HashUtils.h"
typedef struct{
    uint32_t chunk_amount;
    uint32_t chunk_hash_table_size;
    uint32_t current_chunk_index;
    uint64_t total_hash_table_insertions;
    uint64_t total_hash_collisions;
    //Chunk_t* chunks;
    AltChunk_t** chunk_hash_table;
}VoxelGraph_t;

VoxelGraph_t* voxel_graph_init(uint32_t chunk_amount);
void voxel_graph_free(VoxelGraph_t** graph);
bool voxel_graph_insert(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
bool voxel_graph_delete(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
uint8_t voxel_graph_lookup(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
AltChunk_t* voxel_graph_chunk_hash_table_request(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
AltChunk_t* voxel_graph_chunk_hash_table_lookup(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
AltChunk_t* voxel_graph_create_chunk(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);

void voxel_graph_enter_neighbours(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
void voxel_graph_delete_neighbours(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z);
#ifdef __cplusplus
}
#endif

#endif

