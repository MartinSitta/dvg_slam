#include "AltChunk.h"

int64_t alt_chunk_build_anchor_coord(int64_t coord){
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
}
void alt_chunk_init(AltChunk_t* chunk){
    assert(chunk != NULL);
    chunk->x_offset = 0;
    chunk->y_offset = 0;
    chunk->z_offset = 0;
    chunk->chunk_mesh = NULL;
    chunk->change_occurred = false;
    for(uint32_t i = 0; i < ALT_CHUNK_ARR_SIZE; i++){
        chunk->nodes[i] = alt_graph_node_get_blank();
    }
}

void alt_chunk_insert(AltChunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);;;;
    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    assert(rel_x >= 0 && rel_x < ALT_CHUNK_LEN);                                   
    assert(rel_y >= 0 && rel_y < ALT_CHUNK_LEN);                                   
    assert(rel_z >= 0 && rel_z < ALT_CHUNK_LEN);
    uint32_t node_nr = (16*16*rel_z + 16 * rel_y + rel_x);
    uint32_t index = node_nr / 4;
    uint32_t position = node_nr % 4;
    alt_graph_node_set(&(chunk->nodes[index]), position);
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
    uint32_t index = node_nr / 4;
    uint32_t position = node_nr % 4;
    alt_graph_node_delete(&(chunk->nodes[index]), position);
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
    uint32_t index = node_nr / 4;
    uint32_t position = node_nr % 4;
    bool result = alt_graph_node_get(&(chunk->nodes[index]), position);
    return result;
}
