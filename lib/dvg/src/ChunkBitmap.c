#include "ChunkBitmap.h"
void chunk_bitmap_clear_bit(ChunkBitmap_t* bitmap, uint16_t bit_nr){
    assert(bit_nr < 4096);
    assert(bitmap != NULL);
    uint16_t byte_index = bit_nr / 8;
    uint8_t bit_of_byte = bit_nr % 8;
    uint8_t bitmask = ~(1 << bit_of_byte); 
    bitmap->byte_array[byte_index] = bitmap->byte_array[byte_index] & bitmask;
}

void chunk_bitmap_set_bit(ChunkBitmap_t* bitmap, uint16_t bit_nr){
    assert(bit_nr < 4096);
    assert(bitmap != NULL);
    uint16_t byte_index = bit_nr / 8;
    uint8_t bit_of_byte = bit_nr % 8;
    uint8_t bitmask = 1 << bit_of_byte; 
    bitmap->byte_array[byte_index] = bitmap->byte_array[byte_index] | bitmask;

}

bool chunk_bitmap_get_bit(ChunkBitmap_t* bitmap, uint16_t bit_nr){
    assert(bit_nr < 4096);
    assert(bitmap != NULL);
    uint16_t byte_index = bit_nr / 8;
    uint8_t bit_of_byte = bit_nr % 8;
    uint8_t bitmask = (1 << bit_of_byte); 
    return bitmap->byte_array[byte_index] & bitmask;
}

ChunkBitmap_t chunk_bitmap_init(){
    ChunkBitmap_t output;
    for(uint16_t cnt = 0; cnt < 512; cnt++){
        output.byte_array[cnt] = 0;
    }
    return output;
}