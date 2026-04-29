#include "AltChunk.h"

int64_t alt_chunk_build_anchor_coord(int64_t coord){
    /*old implementation
    if(coord >= 0){                                                             
        int64_t ret_val = (coord / ALT_CHUNK_LEN) * ALT_CHUNK_LEN;                    
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
void alt_chunk_init(AltChunk_t* chunk){
    assert(chunk != NULL);
    chunk->x_offset = 0;
    chunk->y_offset = 0;
    chunk->z_offset = 0;
    chunk->change_occurred = false;
    chunk->occupancy_bit_one = alt_graph_bit_map_init();
    chunk->occupancy_bit_two = alt_graph_bit_map_init();
    chunk->inflation = NULL;
}

uint8_t alt_chunk_get_occupancy(AltChunk_t* chunk, uint16_t node_nr){
    assert(chunk != NULL);
    assert(node_nr < ALT_CHUNK_LEN * ALT_CHUNK_LEN * ALT_CHUNK_LEN);
    bool bit_one = alt_graph_bit_map_get_bit(&chunk->occupancy_bit_one, node_nr);
    bool bit_two = alt_graph_bit_map_get_bit(&chunk->occupancy_bit_two, node_nr);
    uint8_t number = bit_one | bit_two << 1;
    return number;
}
void alt_chunk_increase_occupancy(AltChunk_t* chunk, uint16_t node_nr){
    assert(chunk != NULL);
    assert(node_nr < ALT_CHUNK_LEN * ALT_CHUNK_LEN * ALT_CHUNK_LEN);
    bool bit_one = alt_graph_bit_map_get_bit(&chunk->occupancy_bit_one, node_nr);
    bool bit_two = alt_graph_bit_map_get_bit(&chunk->occupancy_bit_two, node_nr);
    uint8_t number = bit_one | bit_two << 1;
    number += 2;
    if(number > 3){
        number = 3;
    }
    bool write_bit_one = number & 1;
    bool write_bit_two = number & 2;
    if(write_bit_one){
        alt_graph_bit_map_set_bit(&chunk->occupancy_bit_one, node_nr);
    }
    else{
        alt_graph_bit_map_clear_bit(&chunk->occupancy_bit_one, node_nr);
    }
    if(write_bit_two){
        alt_graph_bit_map_set_bit(&chunk->occupancy_bit_two, node_nr);
    }
    else{
        alt_graph_bit_map_clear_bit(&chunk->occupancy_bit_two, node_nr);
    }
}
void alt_chunk_decrease_occupancy(AltChunk_t* chunk, uint16_t node_nr){
    assert(chunk != NULL);
    assert(node_nr >= 0);
    assert(node_nr < ALT_CHUNK_LEN * ALT_CHUNK_LEN * ALT_CHUNK_LEN);
    bool bit_one = alt_graph_bit_map_get_bit(&chunk->occupancy_bit_one, node_nr);
    bool bit_two = alt_graph_bit_map_get_bit(&chunk->occupancy_bit_two, node_nr);
    uint8_t number = bit_one | bit_two << 1;
    if(!number){
        return;
    }
    number--;
    bool write_bit_one = number & 1;
    bool write_bit_two = number & 2;
    if(write_bit_one){
        alt_graph_bit_map_set_bit(&chunk->occupancy_bit_one, node_nr);
    }
    else{
        alt_graph_bit_map_clear_bit(&chunk->occupancy_bit_one, node_nr);
    }
    if(write_bit_two){
        alt_graph_bit_map_set_bit(&chunk->occupancy_bit_two, node_nr);
    }
    else{
        alt_graph_bit_map_clear_bit(&chunk->occupancy_bit_two, node_nr);
    }
}


void alt_chunk_insert_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    assert(chunk->inflation != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    alt_graph_bit_map_set_bit(chunk->inflation, node_nr);
}
void alt_chunk_delete_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    assert(chunk->inflation != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    alt_graph_bit_map_clear_bit(chunk->inflation, node_nr);
}
bool alt_chunk_lookup_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    assert(chunk->inflation != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    bool result = alt_graph_bit_map_get_bit(chunk->inflation, node_nr);
    return result;
}
void alt_chunk_insert(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    alt_chunk_increase_occupancy(chunk, node_nr); 
}
void alt_chunk_delete(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    alt_chunk_decrease_occupancy(chunk, node_nr);
}
uint8_t alt_chunk_lookup(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    uint8_t result = alt_chunk_get_occupancy(chunk, node_nr);
    return result;
}