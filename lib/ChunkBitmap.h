#ifndef CHUNKBITMAP_H
#define CHUNKBITMAP_H

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>
typedef struct
{
    uint8_t byte_array[512]; //accounting for 4096 bits accordingly with a 16x16x16 chunk
} ChunkBitmap_t;
void chunk_bitmap_set_bit(ChunkBitmap_t* bitmap, uint16_t number);
void chunk_bitmap_clear_bit(ChunkBitmap_t* bitmap, uint16_t number);
bool chunk_bitmap_get_bit(ChunkBitmap_t* bitmap, uint16_t number);
ChunkBitmap_t chunk_bitmap_init();



#ifdef __cplusplus
}
#endif

#endif