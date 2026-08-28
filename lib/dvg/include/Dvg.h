#ifndef DVG_H
#define DVG_H

#ifdef __cplusplus
extern "C"{
#endif
#include "Chunk.h"
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
    Chunk_t** chunk_hash_table;
}Dvg_t;

Dvg_t* dvg_init(uint32_t chunk_amount);
void dvg_free(Dvg_t** graph);
bool dvg_insert(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
bool dvg_delete(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
uint8_t dvg_lookup(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
Chunk_t* dvg_chunk_hash_table_request(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
Chunk_t* dvg_chunk_hash_table_lookup(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
Chunk_t* dvg_create_chunk(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
void dvg_build_inflation(Dvg_t* graph, int64_t horizontal_inflation, int64_t vertical_inflation);
bool dvg_insert_inflation(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
bool dvg_delete_inflation(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
bool dvg_lookup_inflation(Dvg_t* graph, int64_t x, int64_t y, int64_t z);
#ifdef __cplusplus
}
#endif

#endif

