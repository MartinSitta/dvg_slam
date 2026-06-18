#include "Chunk.h"

int64_t chunk_build_anchor_coord(int64_t coord){
    /*old implementation
    if(coord >= 0){                                                             
        int64_t ret_val = (coord / _CHUNK_LEN) * ALT_CHUNK_LEN;                    
        return ret_val;                                                         
    }                                                                           
    else{                                                                       
        if(coord % ALT_CHUNK_LEN == 0){                                            
            return coord;                                                       
        }                                                                       
        int64_t ret_val = ((coord / ALT_CHUNK_LEN) * ALT_CHUNK_LEN) - (ALT_CHUNK_LEN);   
        return ret_val;                                                         
    }         
    */
    return coord & ~15LL;
}
void chunk_init(Chunk_t* chunk){
    assert(chunk != NULL);
    chunk->x_offset = 0;
    chunk->y_offset = 0;
    chunk->z_offset = 0;
    chunk->change_occurred = false;
    chunk->occupancy_bit_one = chunk_bitmap_init();
    chunk->occupancy_bit_two = chunk_bitmap_init();
    chunk->inflation = NULL;
}
void chunk_free_inflation(Chunk_t* chunk){
    if(chunk->inflation != NULL){
        free(chunk->inflation);
    }
}
uint8_t chunk_get_occupancy(Chunk_t* chunk, uint16_t node_nr){
    assert(chunk != NULL);
    assert(node_nr < ALT_CHUNK_LEN * ALT_CHUNK_LEN * ALT_CHUNK_LEN);
    bool bit_one = chunk_bitmap_get_bit(&chunk->occupancy_bit_one, node_nr);
    bool bit_two = chunk_bitmap_get_bit(&chunk->occupancy_bit_two, node_nr);
    uint8_t number = bit_one | bit_two << 1;
    return number;
}
void chunk_increase_occupancy(Chunk_t* chunk, uint16_t node_nr){
    assert(chunk != NULL);
    assert(node_nr < ALT_CHUNK_LEN * ALT_CHUNK_LEN * ALT_CHUNK_LEN);
    bool bit_one = chunk_bitmap_get_bit(&chunk->occupancy_bit_one, node_nr);
    bool bit_two = chunk_bitmap_get_bit(&chunk->occupancy_bit_two, node_nr);
    uint8_t number = bit_one | bit_two << 1;
    if(number == 3){
        return;
    }
    chunk->change_occurred = true;
    number += 2;
    if(number > 3){
        number = 3;
    }
    bool write_bit_one = number & 1;
    bool write_bit_two = number & 2;
    if(write_bit_one){
        chunk_bitmap_set_bit(&chunk->occupancy_bit_one, node_nr);
    }
    else{
        chunk_bitmap_clear_bit(&chunk->occupancy_bit_one, node_nr);
    }
    if(write_bit_two){
        chunk_bitmap_set_bit(&chunk->occupancy_bit_two, node_nr);
    }
    else{
        chunk_bitmap_clear_bit(&chunk->occupancy_bit_two, node_nr);
    }
}
void chunk_decrease_occupancy(Chunk_t* chunk, uint16_t node_nr){
    assert(chunk != NULL);
    assert(node_nr >= 0);
    assert(node_nr < ALT_CHUNK_LEN * ALT_CHUNK_LEN * ALT_CHUNK_LEN);
    bool bit_one = chunk_bitmap_get_bit(&chunk->occupancy_bit_one, node_nr);
    bool bit_two = chunk_bitmap_get_bit(&chunk->occupancy_bit_two, node_nr);
    uint8_t number = bit_one | bit_two << 1;
    if(!number){
        return;
    }
    chunk->change_occurred = true;
    number--;
    bool write_bit_one = number & 1;
    bool write_bit_two = number & 2;
    if(write_bit_one){
        chunk_bitmap_set_bit(&chunk->occupancy_bit_one, node_nr);
    }
    else{
        chunk_bitmap_clear_bit(&chunk->occupancy_bit_one, node_nr);
    }
    if(write_bit_two){
        chunk_bitmap_set_bit(&chunk->occupancy_bit_two, node_nr);
    }
    else{
        chunk_bitmap_clear_bit(&chunk->occupancy_bit_two, node_nr);
    }
}


void chunk_insert_inflation(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    if(chunk->inflation == NULL){
        ChunkBitmap_t bitmap = chunk_bitmap_init();
        chunk->inflation = malloc(sizeof(ChunkBitmap_t));
        *chunk->inflation = bitmap;
    }
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    chunk_bitmap_set_bit(chunk->inflation, node_nr);
}
void chunk_delete_inflation(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    if(chunk->inflation == NULL){
        ChunkBitmap_t bitmap = chunk_bitmap_init();
        chunk->inflation = malloc(sizeof(ChunkBitmap_t));
        *chunk->inflation = bitmap;
    }
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    chunk_bitmap_clear_bit(chunk->inflation, node_nr);
}
bool chunk_lookup_inflation(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    if(chunk->inflation == NULL){
        return false;
    }
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    bool result = chunk_bitmap_get_bit(chunk->inflation, node_nr);
    return result;
}
void chunk_insert(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    chunk_increase_occupancy(chunk, node_nr); 
}
void chunk_delete(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    chunk_decrease_occupancy(chunk, node_nr);
}
uint8_t chunk_lookup(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    uint8_t result = chunk_get_occupancy(chunk, node_nr);
    return result;
}