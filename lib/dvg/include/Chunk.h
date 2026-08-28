#ifndef CHUNK_H                                                                 
#define CHUNK_H                                                                 
                                                                               
#ifdef __cplusplus                                                              
extern "C" {                                                                    
#endif                                                                          
                                                                                
#include <stdint.h>                                                             
#include "ChunkBitmap.h"
//#include "AltGraphNode.h"                                                          
#include <math.h>                                                               
#include <stddef.h>                                                             
#include <stdbool.h>
#include <assert.h>
#include <stdlib.h>
#define ALT_CHUNK_ARR_SIZE 1024
#define ALT_CHUNK_LEN 16

typedef struct{
    int64_t x_offset;
    int64_t y_offset;
    int64_t z_offset;
    //void* chunk_mesh;
    bool change_occurred;
    ChunkBitmap_t occupancy_bit_one;
    ChunkBitmap_t occupancy_bit_two;
    ChunkBitmap_t* inflation;
    //nodes first two bits contains the occupied and visited bit of each node
    //nodes second two bits contains the clearance bit and a dead bit of each node
    //second bit can be like a temporal layer maybe to show obstacles after a mpping process?
    //mb a mutex bit for future multithreading?
    //or mb it can be used for saving nodes that were there but deleted at one point?

    /*my leading idea is this:
    for the future this can be used as a temporal bit for either a temporal layer like a STVL.
    */
}Chunk_t;

int64_t chunk_build_anchor_coord(int64_t coord);
void chunk_insert_inflation(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
void chunk_delete_inflation(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
bool chunk_lookup_inflation(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
void chunk_free_inflation(Chunk_t* chunk);
void chunk_init(Chunk_t* chunk);
void chunk_insert(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
void chunk_delete(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);
uint8_t chunk_lookup(Chunk_t* chunk, int64_t x, int64_t y, int64_t z);

#ifdef __cplusplus                                                              
}                                                                               
#endif                                                                          
                                                                                
#endif   
