#ifndef CHUNKLISTNODE_H
#define CHUNKLISTNODE_H

#ifdef __cplusplus
extern "C"{
#endif

#include "Chunk.h"

typedef struct ChunkListNode_t ChunkListNode_t;
struct ChunkListNode_t{
    Chunk_t* chunk;
    ChunkListNode_t* prev;
    ChunkListNode_t* next;
};

ChunkListNode_t* chunk_list_node_init(int64_t x, int64_t y, int64_t z);
void chunk_list_node_free(ChunkListNode_t** chunk_list_node);

#ifdef __cplusplus
}
#endif

#endif //CHUNKLISTNODE_H
