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
    chunk->occupancy = alt_graph_bit_map_init();
    chunk->visited = NULL;
    chunk->inflation = NULL;
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
void alt_chunk_lookup_inflation(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
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
    alt_graph_bit_map_set_bit(&chunk->occupancy, node_nr);
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
    alt_graph_bit_map_clear_bit(&chunk->occupancy, node_nr);
}
bool alt_chunk_lookup(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    bool result = alt_graph_bit_map_get_bit(&chunk->occupancy, node_nr);
    return result;
}
