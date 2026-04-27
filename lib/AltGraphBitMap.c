#include "AltGraphBitMap.h"
void alt_graph_bit_map_clear_bit(AltGraphBitMap_t* bitmap, uint16_t bit_nr){
    assert(bit_nr < 4096);
    assert(bitmap != NULL);
    uint16_t byte_index = bit_nr / 8;
    uint8_t bit_of_byte = bit_nr % 8;
    uint8_t bitmask = ~(1 << bit_of_byte); 
    bitmap->byte_array[byte_index] = bitmap->byte_array[byte_index] & bitmask;
}

void alt_graph_bit_map_set_bit(AltGraphBitMap_t* bitmap, uint16_t bit_nr){
    assert(bit_nr < 4096);
    assert(bitmap != NULL);
    uint16_t byte_index = bit_nr / 8;
    uint8_t bit_of_byte = bit_nr % 8;
    uint8_t bitmask = 1 << bit_of_byte; 
    bitmap->byte_array[byte_index] = bitmap->byte_array[byte_index] | bitmask;

}

bool alt_graph_bit_map_get_bit(AltGraphBitMap_t* bitmap, uint16_t bit_nr){
    assert(bit_nr < 4096);
    assert(bitmap != NULL);
    uint16_t byte_index = bit_nr / 8;
    uint8_t bit_of_byte = bit_nr % 8;
    uint8_t bitmask = (1 << bit_of_byte); 
    return bitmap->byte_array[byte_index] & bitmask;
}

AltGraphBitMap_t alt_graph_bit_map_init(){
    AltGraphBitMap_t output;
    for(uint16_t cnt = 0; cnt < 512; cnt++){
        output.byte_array[cnt] = 0;
    }
    return output;
}