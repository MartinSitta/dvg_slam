#include "VoxelGraph.h"
#include <assert.h>
#include "HashUtils.h"
#include <stdio.h>
//helper functions
void voxel_graph_init_arrays(VoxelGraph_t* graph){
    for(uint32_t i = 0; i < graph->chunk_hash_table_size; i++){
        graph->chunk_hash_table[i] = NULL;
    }
}

//end helpers
VoxelGraph_t* voxel_graph_init(uint32_t chunk_count){
    assert(chunk_count > 0);
    assert((chunk_count & (chunk_count - 1)) == 0);
    uint32_t chunk_amount = chunk_count;
    uint32_t chunk_hash_table_size = chunk_count * 2;
    VoxelGraph_t* output = malloc(sizeof(VoxelGraph_t));
    output->chunk_amount = chunk_count;
    //output->chunks = NULL;
    output->chunk_hash_table = NULL;
    output->current_chunk_index = 0;
    if(((chunk_hash_table_size) & (chunk_hash_table_size - 1)) != 0){
        bool val_found = false;
        for(uint32_t i = 1; (i < (i << 31)) && !val_found; i << 1){
            if(i > chunk_hash_table_size){
                chunk_hash_table_size = i;
                val_found = true;
            }
        }

    }
    output->chunk_hash_table = malloc(sizeof(Chunk_t*) * chunk_hash_table_size);
    assert(output->chunk_hash_table != NULL);
    output->chunk_hash_table_size = chunk_hash_table_size;
    printf("chunk hashtable size is %d\n", output->chunk_hash_table_size);
    assert(output->chunk_hash_table_size > output->chunk_amount);
    voxel_graph_init_arrays(output);
    return output;
}
Vertex_t* voxel_graph_get_vertex(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    assert(graph != NULL);
    Chunk_t* chunk = voxel_graph_chunk_hash_table_lookup(graph, x, y, z);
    if(chunk == NULL){
        return NULL;
    }
    assert(build_anchor_coord(x) == chunk->x_offset);                           
    assert(build_anchor_coord(y) == chunk->y_offset);                           
    assert(build_anchor_coord(z) == chunk->z_offset);
    
    int16_t rel_x = x - chunk->x_offset;                                        
    int16_t rel_y = y - chunk->y_offset;                                        
    int16_t rel_z = z - chunk->z_offset;                                        
    
    uint16_t node_coords = build_vertex_coords((uint8_t) rel_x, (uint8_t) rel_y, (uint8_t) rel_z);
    int64_t node_index = chunk_node_lookup(chunk, node_coords);
    if(node_index == -1){
        return NULL;
    }
    Vertex_t* out_ptr = &chunk->nodes[node_index].coord_and_mesh_info;
    assert(out_ptr->vertex_coords == node_coords);
    return out_ptr;
}
bool voxel_graph_insert(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    assert(graph != NULL);
    //printf("performing hash table lookup\n");
    Chunk_t* chunk = voxel_graph_chunk_hash_table_request(graph, x, y, z);
    if(chunk == NULL){
        return false;
    }
    assert(graph->current_chunk_index < graph->chunk_amount);
    if(graph->current_chunk_index < 0){
        return false;
    }
    assert(chunk->x_offset == build_anchor_coord(x));
    assert(chunk->y_offset == build_anchor_coord(y));
    assert(chunk->z_offset == build_anchor_coord(z));
    //printf("performing chunk_insertion\n");
    bool ret_val = chunk_insert(chunk,x, y, z);
    if(ret_val){
        voxel_graph_enter_neighbours(graph, x, y, z);
    }
    return ret_val;
}
bool voxel_graph_delete(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    assert(graph != NULL);
    Chunk_t* chunk = voxel_graph_chunk_hash_table_lookup(graph, x, y, z);
    if(chunk == NULL){
        return false;
    }
    assert(chunk->x_offset == build_anchor_coord(x));
    assert(chunk->y_offset == build_anchor_coord(y));
    assert(chunk->z_offset == build_anchor_coord(z));
    bool ret_val = chunk_delete(chunk,x, y, z);
    if(ret_val){
        voxel_graph_delete_neighbours(graph, x, y, z);
    }
    return ret_val;
}
Chunk_t* voxel_graph_chunk_hash_table_lookup(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    int64_t chunk_anchor_x = build_anchor_coord(x);
    int64_t chunk_anchor_y = build_anchor_coord(y);
    int64_t chunk_anchor_z = build_anchor_coord(z);
    //printf("building hash\n");
    uint32_t hash_table_entry =  build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 1586102333);
    hash_table_entry = hash_table_entry & (graph->chunk_hash_table_size - 1);
    assert(hash_table_entry < graph->chunk_hash_table_size);
    //printf("building double_hash\n");
    uint64_t double_hash = 0;
    uint64_t double_hash_val = build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 2734158491);
    //printf("performing double hash hashtable lookups\n");
    for(uint32_t i = 0; i < 10; i++){
        double_hash = ((uint64_t)hash_table_entry + ((i * double_hash_val))) % graph->chunk_hash_table_size;
        assert(double_hash < graph->chunk_hash_table_size);
        if(graph->chunk_hash_table[double_hash] == NULL){
            return NULL;
        }
        else{
            Chunk_t* chunk = graph->chunk_hash_table[double_hash]; 
            if(chunk->x_offset == chunk_anchor_x &&
                    chunk->y_offset == chunk_anchor_y &&
                    chunk->z_offset == chunk_anchor_z){
                return chunk;
            }
        }
    }
    return NULL;
}
Chunk_t* voxel_graph_chunk_hash_table_request(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    int64_t chunk_anchor_x = build_anchor_coord(x);
    int64_t chunk_anchor_y = build_anchor_coord(y);
    int64_t chunk_anchor_z = build_anchor_coord(z);
    //printf("building hash\n");
    uint32_t hash_table_entry =  build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 1586102333);
    hash_table_entry = hash_table_entry & (graph->chunk_hash_table_size - 1);
    assert(hash_table_entry < graph->chunk_hash_table_size);
    //printf("performing intial hashtable lookup\n");
    if(graph->chunk_hash_table[hash_table_entry] == NULL){
        Chunk_t* chunk = voxel_graph_create_chunk(graph, x, y, z);
        if(chunk == NULL){
            return NULL;
        }
        graph->chunk_hash_table[hash_table_entry] = chunk;
        return chunk;
    }
    else{
        //printf("building double_hash\n");
        uint64_t double_hash = 0;
        uint64_t double_hash_val = build_chunk_hash_table_hash(chunk_anchor_x, chunk_anchor_y, chunk_anchor_z, 2734158491);
        //printf("performing double hash hashtable lookups\n");
        for(uint32_t i = 0; i < 10; i++){
            double_hash = ((uint64_t)hash_table_entry + ((i * double_hash_val))) % graph->chunk_hash_table_size;
            if(graph->chunk_hash_table[double_hash] == NULL){
                Chunk_t* chunk = voxel_graph_create_chunk(graph, x, y, z);
                if(chunk == NULL){
                    return NULL;
                }
                graph->chunk_hash_table[double_hash] = chunk;
                return chunk;
            }
            else{
                Chunk_t* chunk = graph->chunk_hash_table[double_hash]; 
                if(chunk->x_offset == chunk_anchor_x &&
                        chunk->y_offset == chunk_anchor_y &&
                        chunk->z_offset == chunk_anchor_z){
                    return chunk;
                }
            }
        }
        
    return NULL;
    }
}
Chunk_t* voxel_graph_create_chunk(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    assert(graph != NULL);
    if(graph == NULL){
        return NULL;
    }
    if(graph->current_chunk_index >= graph->chunk_amount){
        return NULL;
    }
    Chunk_t* chunk = malloc(sizeof(Chunk_t));
    if(chunk == NULL){
        return NULL;
    }
    chunk_init(chunk);
    chunk->x_offset = build_anchor_coord(x);
    chunk->y_offset = build_anchor_coord(y);
    chunk->z_offset = build_anchor_coord(z);
    graph->current_chunk_index++;
    return chunk;
}

void voxel_graph_enter_neighbours(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    Vertex_t* org_vertex = voxel_graph_get_vertex(graph, x, y, z);

    Vertex_t* upper_vertex = voxel_graph_get_vertex(graph, x, y, z + 1);            
    Vertex_t* lower_vertex = voxel_graph_get_vertex(graph, x, y, z - 1);            
    Vertex_t* left_vertex = voxel_graph_get_vertex(graph, x, y + 1, z);             
    Vertex_t* right_vertex = voxel_graph_get_vertex(graph, x, y - 1, z);            
    Vertex_t* foward_vertex = voxel_graph_get_vertex(graph, x + 1, y, z);            
    Vertex_t* back_vertex = voxel_graph_get_vertex(graph, x - 1, y, z);

    assert(org_vertex != NULL);                                                 
    if(upper_vertex != NULL){
        if(!vertex_get_dead_bit(upper_vertex)){
            vertex_set_up_bit(org_vertex);
            vertex_set_down_bit(upper_vertex);
        }
    }
    if(lower_vertex != NULL){                                                   
        if(!vertex_get_dead_bit(lower_vertex)){
            vertex_set_down_bit(org_vertex);
            vertex_set_up_bit(lower_vertex);
        }
    }                                                                           
    if(left_vertex != NULL){                                                    
        if(!vertex_get_dead_bit(left_vertex)){
            vertex_set_left_bit(org_vertex);
            vertex_set_right_bit(left_vertex);
        }
    }                                                                           
    if(right_vertex != NULL){                                                   
        if(!vertex_get_dead_bit(right_vertex)){
            vertex_set_right_bit(org_vertex);
            vertex_set_left_bit(right_vertex);
        }
    }                                                                           
    if(foward_vertex != NULL){                                                  
        if(!vertex_get_dead_bit(foward_vertex)){
            vertex_set_foward_bit(org_vertex);
            vertex_set_back_bit(foward_vertex);
        }
    }                                                                           
    if(back_vertex != NULL){                                                    
        if(!vertex_get_dead_bit(back_vertex)){
            vertex_set_back_bit(org_vertex);
            vertex_set_foward_bit(back_vertex);
        }
    }                
}

void voxel_graph_delete_neighbours(VoxelGraph_t* graph, int64_t x, int64_t y, int64_t z){
    Vertex_t* org_vertex = voxel_graph_get_vertex(graph, x, y, z);
    Vertex_t* upper_vertex = voxel_graph_get_vertex(graph, x, y, z + 1);
    Vertex_t* lower_vertex = voxel_graph_get_vertex(graph, x, y, z - 1);
    Vertex_t* left_vertex = voxel_graph_get_vertex(graph, x, y + 1, z);
    Vertex_t* right_vertex = voxel_graph_get_vertex(graph, x, y - 1, z);
    Vertex_t* foward_vertex = voxel_graph_get_vertex(graph, x + 1, y, z);            
    Vertex_t* back_vertex = voxel_graph_get_vertex(graph, x - 1, y, z);            
    assert(org_vertex != NULL);
    if(upper_vertex != NULL){
        vertex_clear_up_bit(org_vertex);
        vertex_clear_down_bit(upper_vertex);
        assert(!vertex_get_up_bit(org_vertex));
        assert(!vertex_get_down_bit(upper_vertex));
    }
    if(lower_vertex != NULL){
        vertex_clear_down_bit(org_vertex);
        vertex_clear_up_bit(lower_vertex);
        assert(!vertex_get_down_bit(org_vertex));
        assert(!vertex_get_up_bit(lower_vertex));
    }
    if(left_vertex != NULL){
        vertex_clear_left_bit(org_vertex);
        vertex_clear_right_bit(left_vertex);
        assert(!vertex_get_left_bit(org_vertex));
        assert(!vertex_get_right_bit(left_vertex));
    }
    if(right_vertex != NULL){
        vertex_clear_right_bit(org_vertex);
        vertex_clear_left_bit(right_vertex);
        assert(!vertex_get_right_bit(org_vertex));
        assert(!vertex_get_left_bit(right_vertex));
    }
    if(foward_vertex != NULL){
        vertex_clear_foward_bit(org_vertex);
        vertex_clear_back_bit(foward_vertex);
        assert(!vertex_get_foward_bit(org_vertex));
        assert(!vertex_get_back_bit(foward_vertex));
    }
    if(back_vertex != NULL){
        vertex_clear_back_bit(org_vertex);
        vertex_clear_foward_bit(back_vertex);
        assert(!vertex_get_back_bit(org_vertex));
        assert(!vertex_get_foward_bit(back_vertex));
    }

}

void voxel_graph_free(VoxelGraph_t** graph){
    for(uint32_t i = 0; i < (*graph)->chunk_hash_table_size; i++){
        if((*graph)->chunk_hash_table[i] != NULL){
            free((*graph)->chunk_hash_table[i]);
            (*graph)->chunk_hash_table[i] = NULL;
        }
    }
    free((*graph)->chunk_hash_table);
    free(*graph);
    *graph = NULL;
}






