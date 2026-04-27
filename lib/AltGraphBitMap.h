#ifndef ATLGRAPHBITMAP_H
#define ALTGRAPHBITMAP_H

#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stddef.h>
typedef struct AltGraphBitMap
{
    uint8_t byte_array[512]; //accounting for 4096 bits accordingly with a 16x16x16 chunk
} AltGraphBitMap_t;
void alt_graph_bit_map_set_bit(AltGraphBitMap_t* bitmap, uint16_t number);
void alt_graph_bit_map_clear_bit(AltGraphBitMap_t* bitmap, uint16_t number);
bool alt_graph_bit_map_get_bit(AltGraphBitMap_t* bitmap, uint16_t number);
AltGraphBitMap_t alt_graph_bit_map_init();



#ifdef __cplusplus
}
#endif

#endif