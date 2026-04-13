#ifndef ALTCHUNK_H                                                                 
#define ALTCHUNK_H                                                                 
                                                                               
#ifdef __cplusplus                                                              
extern "C" {                                                                    
#endif                                                                          
                                                                                
#include <stdint.h>                                                             
#include "AltGraphNode.h"                                                          
#include <math.h>                                                               
#include <stddef.h>                                                             
#include <stdbool.h>
#include <assert.h>

#define ALT_CHUNK_ARR_SIZE 1024
#define ALT_CHUNK_LEN 16

typedef struct{
    int64_t x_offset;
    int64_t y_offset;
    int64_t z_offset;
    void* chunk_mesh;
    bool change_occurred;
    AltGraphNode_t nodes[1024];//4 graph nodes fit into one byte. 16x16x16 chunk
}AltChunk_t;

int64_t alt_chunk_build_anchor_coord(int64_t coord);
void alt_chunk_init(AltChunk_t* chunk);
void alt_chunk_insert(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
void alt_chunk_delete(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
bool alt_chunk_lookup(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);

#ifdef __cplusplus                                                              
}                                                                               
#endif                                                                          
                                                                                
#endif   
