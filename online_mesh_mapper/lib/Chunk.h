#ifndef CHUNK_H
#define CHUNK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "GraphNode.h"
#include "Face.h"
#include "GraphDefines.h"
#include "Vertex.h"
#include "HashUtils.h"
#include <math.h>
#include <stddef.h>
#include <stdbool.h>
typedef struct{
    int64_t x_offset;
    int64_t y_offset;
    int64_t z_offset;
    uint16_t current_node_index;
    GraphNode_t nodes[CHUNK_NODE_LIMIT];
    chunk_node_array_entry_t node_hash_table[CHUNK_NODE_HASHTABLE_SIZE];
}Chunk_t;

Chunk_t* chunk_init();
bool chunk_insert( Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
bool chunk_delete(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
uint8_t chunk_request_node(Chunk_t* chunk, uint16_t coords); //I have to change the return type to uint8_t cause I need more output codes
//status codes:
//0: request failed
//1: node successfully added
//2: node was a duplicate
int64_t chunk_node_lookup(Chunk_t* chunk, uint16_t coords);
void chunk_node_enter_neighbours(Chunk_t* chunk, chunk_node_array_entry_t index);
void chunk_node_delete_neighbours(Chunk_t* chunk, chunk_node_array_entry_t index);
int64_t chunk_create_node(Chunk_t* chunk, uint16_t coords);
bool chunk_delete_node(Chunk_t* chunk, uint16_t index);
int64_t build_anchor_coord(int64_t coords);

void chunk_node_connect_upper_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_connect_lower_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_connect_left_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_connect_right_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_connect_foward_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_connect_back_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);

void chunk_node_disconnect_upper_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_disconnect_lower_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_disconnect_left_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_disconnect_right_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_disconnect_foward_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);
void chunk_node_disconnect_back_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index, chunk_node_array_entry_t dest_index);


#ifdef __cplusplus
}
#endif

#endif
