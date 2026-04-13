#ifndef CHUNKDLL_H
#define CHUNKDLL_H

#ifdef __cplusplus
extern "C"{
#endif

#include "ChunkListNode.h"

ChunkListNode_t* chunk_dll_insert_at_start(ChunkListNode_t** head, int64_t x, int64_t y, int64_t z);
bool chunk_dll_delete_at_start(ChunkListNode_t** head);
void chunk_dll_free(ChunkListNode_t** head);
uint8_t chunk_dll_delete_by_pointer(ChunkListNode_t** head, ChunkListNode_t** ptr);
#ifdef __cplusplus
}
#endif

#endif //CHUNKDLL_H
