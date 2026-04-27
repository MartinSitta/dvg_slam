#ifndef ALTCHUNK_H                                                                 
#define ALTCHUNK_H                                                                 
                                                                               
#ifdef __cplusplus                                                              
extern "C" {                                                                    
#endif                                                                          
                                                                                
#include <stdint.h>                                                             
#include "AltGraphBitMap.h"
//#include "AltGraphNode.h"                                                          
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
    //void* chunk_mesh;
    bool change_occurred;
    AltGraphBitMap_t occupancy;
    AltGraphBitMap_t* visited;
    AltGraphBitMap_t* inflation;
    //nodes first two bits contains the occupied and visited bit of each node
    //nodes second two bits contains the clearance bit and a dead bit of each node
    //second bit can be like a temporal layer maybe to show obstacles after a mpping process?
    //mb a mutex bit for future multithreading?
    //or mb it can be used for saving nodes that were there but deleted at one point?

    /*my leading idea is this:
    for the future this can be used as a temporal bit for either a temporal layer like a STVL.
    */
}AltChunk_t;

int64_t alt_chunk_build_anchor_coord(int64_t coord);
void alt_chunk_insert_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
void alt_chunk_delete_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
void alt_chunk_lookup_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
void alt_chunk_init(AltChunk_t* chunk);
void alt_chunk_insert(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
void alt_chunk_delete(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);
bool alt_chunk_lookup(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z);

#ifdef __cplusplus                                                              
}                                                                               
#endif                                                                          
                                                                                
#endif   
