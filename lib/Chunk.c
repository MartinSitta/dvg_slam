#include "Chunk.h"
#include "GraphNode.h"
#include <assert.h>
#include <stddef.h>

void chunk_node_connect_upper_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    if(!vertex_get_dead_bit(&(chunk->nodes[dest_index].coord_and_mesh_info))){
        vertex_set_up_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
        vertex_set_down_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
    }
}

void chunk_node_connect_lower_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    if(!vertex_get_dead_bit(&(chunk->nodes[dest_index].coord_and_mesh_info))){
        vertex_set_down_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
        vertex_set_up_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
    }
}

void chunk_node_connect_left_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    if(!vertex_get_dead_bit(&(chunk->nodes[dest_index].coord_and_mesh_info))){
        vertex_set_left_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
        vertex_set_right_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
    }
}

void chunk_node_connect_right_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    if(!vertex_get_dead_bit(&(chunk->nodes[dest_index].coord_and_mesh_info))){
        vertex_set_right_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
        vertex_set_left_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
    }
}

void chunk_node_connect_foward_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    if(!vertex_get_dead_bit(&(chunk->nodes[dest_index].coord_and_mesh_info))){
        vertex_set_foward_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
        vertex_set_back_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
    }
}

void chunk_node_connect_back_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    if(!vertex_get_dead_bit(&(chunk->nodes[dest_index].coord_and_mesh_info))){
        vertex_set_back_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
        vertex_set_foward_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
    }
}


void chunk_node_disconnect_upper_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    vertex_clear_up_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
    vertex_clear_down_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
}

void chunk_node_disconnect_lower_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    vertex_clear_down_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
    vertex_clear_up_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
}

void chunk_node_disconnect_left_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    vertex_clear_left_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
    vertex_clear_right_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
}

void chunk_node_disconnect_right_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    vertex_clear_right_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
    vertex_clear_left_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
}

void chunk_node_disconnect_foward_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    vertex_clear_foward_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
    vertex_clear_back_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
}

void chunk_node_disconnect_back_neighbour(Chunk_t* chunk, chunk_node_array_entry_t org_index,
        chunk_node_array_entry_t dest_index){
    assert(chunk != NULL);
    assert(org_index >= 0);
    assert(dest_index >= 0);
    vertex_clear_back_bit(&(chunk->nodes[org_index].coord_and_mesh_info));
    vertex_clear_foward_bit(&(chunk->nodes[dest_index].coord_and_mesh_info));
}



void chunk_lookup_and_connect_nodes_helper(Chunk_t* chunk,
        chunk_node_array_entry_t org_index, uint16_t neighbours[6]){
    assert(chunk != NULL);
    assert(org_index >= 0);
    if(neighbours[0] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[0]);
        if(index >= 0){
            chunk_node_connect_upper_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[1] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[1]);
        if(index >= 0){
            chunk_node_connect_lower_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[2] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[2]);
        if(index >= 0){
            chunk_node_connect_left_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[3] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[3]);
        if(index >= 0){
            chunk_node_connect_right_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[4] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[4]);
        if(index >= 0){
            chunk_node_connect_foward_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[5] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[5]);
        if(index >= 0){
            chunk_node_connect_back_neighbour(chunk, org_index, index);
        }
    }
}

void chunk_lookup_and_delete_nodes_helper(Chunk_t* chunk,
        chunk_node_array_entry_t org_index, uint16_t neighbours[6]){
    assert(chunk != NULL);
    assert(org_index >= 0);
    if(neighbours[0] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[0]);
        if(index >= 0){
            chunk_node_disconnect_upper_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[1] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[1]);
        if(index >= 0){
            chunk_node_disconnect_lower_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[2] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[2]);
        if(index >= 0){
            chunk_node_disconnect_left_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[3] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[3]);
        if(index >= 0){
            chunk_node_disconnect_right_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[4] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[4]);
        if(index >= 0){
            chunk_node_disconnect_foward_neighbour(chunk, org_index, index);
        }
    }
    if(neighbours[5] != 1){
        chunk_node_array_entry_t index = chunk_node_lookup(chunk, neighbours[5]);
        if(index >= 0){
            chunk_node_disconnect_back_neighbour(chunk, org_index, index);
        }
    }
}


Chunk_t* chunk_init(Chunk_t* chunk){
    chunk->x_offset = 0;
    chunk->y_offset = 0;
    chunk->z_offset = 0;
    chunk->current_node_index = 0;
    for(uint32_t i = 0; i < CHUNK_NODE_LIMIT; i++){
        chunk->nodes[i] = graph_node_init();
    }
    for(uint32_t i = 0; i < CHUNK_NODE_HASHTABLE_SIZE; i++){
        chunk->node_hash_table[i] = -1;
    }
    assert(chunk->nodes[0].coord_and_mesh_info.buf[0] == 0);
    assert(chunk->nodes[0].coord_and_mesh_info.buf[1] == 0);
    assert(chunk->nodes[0].coord_and_mesh_info.buf[2] == 0);
    //assert(output.nodes[0].upper_neighbour == -1);
    //assert(output.nodes[0].back_neighbour == -1);

    return chunk;
}

void chunk_node_enter_neighbours(Chunk_t* chunk, chunk_node_array_entry_t index){
    assert(chunk != NULL);
    assert(index >= 0);
    Vertex_t org_vertex = chunk->nodes[index].coord_and_mesh_info;
    uint8_t org_x_coord = vertex_pick_x_coord(org_vertex.vertex_coords); 
    uint8_t org_y_coord = vertex_pick_y_coord(org_vertex.vertex_coords); 
    uint8_t org_z_coord = vertex_pick_z_coord(org_vertex.vertex_coords); 
    uint16_t neighbour_indeces[6] = {1,1,1,1,1,1}; // up down left right foward back
    if(org_z_coord < 31){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord, org_z_coord + 1);
        neighbour_indeces[0] = test_coord;
    }
    if(org_z_coord > 0){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord, org_z_coord - 1);
        neighbour_indeces[1] = test_coord;
    }
    if(org_y_coord < 31){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord + 1, org_z_coord);
        neighbour_indeces[2] = test_coord;
    }
    if(org_y_coord > 0){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord - 1, org_z_coord);
        neighbour_indeces[3] = test_coord;
    }
    if(org_x_coord < 31){
        uint16_t test_coord = build_vertex_coords(org_x_coord + 1, org_y_coord, org_z_coord);
        neighbour_indeces[4] = test_coord;
    }
    if(org_x_coord > 0){
        uint16_t test_coord = build_vertex_coords(org_x_coord - 1, org_y_coord, org_z_coord);
        neighbour_indeces[5] = test_coord;
    }
    chunk_lookup_and_connect_nodes_helper(chunk, index, neighbour_indeces);
}


void chunk_node_delete_neighbours(Chunk_t* chunk, chunk_node_array_entry_t index){
    assert(chunk != NULL);
    assert(index >= 0);
    Vertex_t org_vertex = chunk->nodes[index].coord_and_mesh_info;
    uint8_t org_x_coord = vertex_pick_x_coord(org_vertex.vertex_coords); 
    uint8_t org_y_coord = vertex_pick_y_coord(org_vertex.vertex_coords); 
    uint8_t org_z_coord = vertex_pick_z_coord(org_vertex.vertex_coords); 
    uint16_t neighbour_indeces[6] = {1,1,1,1,1,1}; // up down left right foward back
    if(org_z_coord < 31){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord, org_z_coord + 1);
        neighbour_indeces[0] = test_coord;
    }
    if(org_z_coord > 0){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord, org_z_coord - 1);
        neighbour_indeces[1] = test_coord;
    }
    if(org_y_coord < 31){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord + 1, org_z_coord);
        neighbour_indeces[2] = test_coord;
    }
    if(org_y_coord > 0){
        uint16_t test_coord = build_vertex_coords(org_x_coord, org_y_coord - 1, org_z_coord);
        neighbour_indeces[3] = test_coord;
    }
    if(org_x_coord < 31){
        uint16_t test_coord = build_vertex_coords(org_x_coord + 1, org_y_coord, org_z_coord);
        neighbour_indeces[4] = test_coord;
    }
    if(org_x_coord > 0){
        uint16_t test_coord = build_vertex_coords(org_x_coord - 1, org_y_coord, org_z_coord);
        neighbour_indeces[5] = test_coord;
    }
    chunk_lookup_and_delete_nodes_helper(chunk, index, neighbour_indeces);
}


bool chunk_insert(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    if(chunk == NULL){
        return false;
    }
    assert(build_anchor_coord(x) == chunk->x_offset);
    assert(build_anchor_coord(y) == chunk->y_offset);
    assert(build_anchor_coord(z) == chunk->z_offset);

    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;

    assert(rel_x >= 0 && rel_x < CHUNK_SIZE);
    assert(rel_y >= 0 && rel_y < CHUNK_SIZE);
    assert(rel_z >= 0 && rel_z < CHUNK_SIZE);

    uint16_t coords = build_vertex_coords((uint8_t) rel_x, (uint8_t)rel_y, (uint8_t)rel_z);
    uint8_t status_code = chunk_request_node(chunk, coords);
    if(status_code == 1){
        chunk_node_array_entry_t org_index = chunk->current_node_index -1;
        return true;
    }
    else if(status_code == 2){
        return true;
    }
    else{
        return false;
    }
}
bool chunk_delete(Chunk_t* chunk, int64_t x, int64_t y, int64_t z){
    assert(chunk != NULL);
    if(chunk == NULL){
        return false;
    }
    assert(build_anchor_coord(x) == chunk->x_offset);
    assert(build_anchor_coord(y) == chunk->y_offset);
    assert(build_anchor_coord(z) == chunk->z_offset);

    int16_t rel_x = x - chunk->x_offset;
    int16_t rel_y = y - chunk->y_offset;
    int16_t rel_z = z - chunk->z_offset;
    
    assert(rel_x >= 0 && rel_x < CHUNK_SIZE);
    assert(rel_y >= 0 && rel_y < CHUNK_SIZE);
    assert(rel_z >= 0 && rel_z < CHUNK_SIZE);
    
    uint16_t coords = build_vertex_coords((uint8_t) rel_x, (uint8_t)rel_y, (uint8_t)rel_z);
    int64_t index = chunk_node_lookup(chunk, coords);
    if(index != -1){
        chunk_delete_node(chunk, (uint16_t) index);
        return true;
    }
    else{
        return false;
    }
}

uint8_t chunk_request_node(Chunk_t* chunk, uint16_t coords){
    assert(chunk != NULL);
    uint16_t hash = build_node_hash_table_hash(coords, 1586102333);
    hash = hash & (CHUNK_NODE_HASHTABLE_SIZE - 1);
    uint64_t double_hash = build_node_hash_table_hash(coords, 273415849);
    for(uint32_t i = 0; i < 10; i++){
        uint16_t new_hash = (((uint64_t)hash) + (i * double_hash)) % CHUNK_NODE_HASHTABLE_SIZE;
        assert(new_hash < CHUNK_NODE_HASHTABLE_SIZE);
        chunk_node_array_entry_t entry = chunk->node_hash_table[new_hash];
        if(entry == -1){
            int64_t new_index = chunk_create_node(chunk, coords);
            if(new_index != -1){
                chunk->node_hash_table[new_hash] = new_index;
                return 1;
            }
            else{
                return 0;
            }
        }
        else if(chunk->nodes[entry].coord_and_mesh_info.vertex_coords == coords){
            if(vertex_get_dead_bit(&chunk->nodes[entry].coord_and_mesh_info)){
                vertex_clear_dead_bit(&chunk->nodes[entry].coord_and_mesh_info);
            }
            return 2;
        }
    }
    return 0;
}

int64_t chunk_node_lookup(Chunk_t* chunk, uint16_t coords){
    assert(chunk != NULL);
    uint16_t hash = build_node_hash_table_hash(coords, 1586102333);
    hash = hash & (CHUNK_NODE_HASHTABLE_SIZE - 1);
    uint64_t double_hash = build_node_hash_table_hash(coords, 273415849);
    for(uint32_t i = 0; i < 10; i++){
        uint16_t new_hash = (((uint64_t)hash) + (i * double_hash)) % CHUNK_NODE_HASHTABLE_SIZE;

        assert(new_hash < CHUNK_NODE_HASHTABLE_SIZE);
        chunk_node_array_entry_t entry = chunk->node_hash_table[new_hash];
        if(entry == -1){
            return -1;
        }
        else{
            if(chunk->nodes[entry].coord_and_mesh_info.vertex_coords == coords){
                return entry;
            }
        }
    }
    return -1;
}

int64_t chunk_create_node(Chunk_t* chunk, uint16_t coords){
    assert(chunk != NULL);
    assert(chunk->nodes[chunk->current_node_index].coord_and_mesh_info.vertex_coords == 0);
    if(chunk->current_node_index >= CHUNK_NODE_LIMIT){
        return -1;
    }
    int64_t return_val = chunk->current_node_index;
    chunk->nodes[chunk->current_node_index].coord_and_mesh_info.buf[2] = 0;
    chunk->nodes[chunk->current_node_index].coord_and_mesh_info.vertex_coords = coords;
    chunk->current_node_index++;
    return return_val;
}

bool chunk_delete_node(Chunk_t* chunk, uint16_t index){
    assert(chunk != NULL);
    if(vertex_get_dead_bit(&chunk->nodes[index].coord_and_mesh_info)){
        return false;
    }
    else{
        uint16_t coords = chunk->nodes[index].coord_and_mesh_info.vertex_coords;
        vertex_set_dead_bit(&chunk->nodes[index].coord_and_mesh_info);
        return true;
    }
}
int64_t build_anchor_coord(int64_t input){
    if(input >= 0){
        int64_t ret_val = (input / CHUNK_SIZE) * CHUNK_SIZE;
        return ret_val;
    }
    else{
        if(input % CHUNK_SIZE == 0){
            return input;
        }
        int64_t ret_val = ((input / CHUNK_SIZE) * CHUNK_SIZE) - (CHUNK_SIZE);
        return ret_val;
    }
}
