#include <stdio.h>
#include "Chunk.h"
#include "VoxelGraph.h"
#include "HashUtils.h"

int main(){
    printf("%ld\n", sizeof(GraphNode_t));
    printf("%ld\n", sizeof(AltChunk_t));
    VoxelGraph_t* graph = voxel_graph_init(1<<15);
    printf("graph chunk_array_size is %ld \n",(long) graph->chunk_amount);
    
    printf("graph chunk_hash_table_size is %ld \n", (long) graph->chunk_hash_table_size);
    printf("size of base graph is %ld\n", sizeof(*graph) + sizeof((graph->chunk_hash_table[0])) * graph->chunk_hash_table_size);
    printf("hash-test 1: %d\n", build_chunk_hash_table_hash(2354, 123, 4, 032540327));

    uint16_t coords = build_vertex_coords(16, 16, 16);
    printf("coords sanity check %d\n", coords);
    printf("%d\n%d\n%d\n", vertex_pick_x_coord(coords), vertex_pick_y_coord(coords), vertex_pick_z_coord(coords));

    printf("anchor coord sanity test\n");
    printf("%ld\n", build_anchor_coord(32));
    printf("%ld\n", build_anchor_coord(0));
    printf("%ld\n", build_anchor_coord(-1));

    printf("hash-test 2: %d\n", build_node_hash_table_hash(coords, 032540327));
    voxel_graph_insert(graph, 1, 1, 1);
    AltChunk_t* chunk = voxel_graph_chunk_hash_table_lookup(graph, 1, 1, 1);
    assert(graph->current_chunk_index == 1);
    assert(chunk->x_offset == 0);
    assert(chunk->y_offset == 0);
    assert(chunk->z_offset == 0);
    //assert(chunk->current_node_index == 1);
    //assert(chunk->nodes[0].coord_and_mesh_info.vertex_coords != 0);
    assert(alt_chunk_lookup(chunk, 1, 1, 1));
    voxel_graph_insert(graph, 2, 2, 2);
    assert(graph->current_chunk_index == 1);
    assert(chunk->x_offset == 0);
    assert(chunk->y_offset == 0);
    assert(chunk->z_offset == 0);
    //assert(chunk->current_node_index == 2);
    //assert(chunk->nodes[1].coord_and_mesh_info.vertex_coords != 0);
    assert(alt_chunk_lookup(chunk, 2, 2, 2));
    voxel_graph_insert(graph, 2, 2, 2);
    assert(graph->current_chunk_index == 1);
    assert(chunk->x_offset == 0);
    assert(chunk->y_offset == 0);
    assert(chunk->z_offset == 0);
    //assert(chunk->current_node_index == 2);

    printf("testing chunk insertion with negative coords\n");
    voxel_graph_insert(graph, 2, -1, 2);
    AltChunk_t* chunk_two = voxel_graph_chunk_hash_table_lookup(graph, 2, -1, 2);
    assert(graph->current_chunk_index == 2);
    assert(chunk_two->x_offset == 0);
    assert(chunk_two->y_offset == -16);
    assert(chunk_two->z_offset == 0);
    //assert(chunk_two->current_node_index == 1);

    printf("depricated test: neighbour connection sanity check:\n");
    voxel_graph_insert(graph, 1, 1, 2);
    assert(graph->current_chunk_index == 2);
    assert(chunk->x_offset == 0);
    assert(chunk->y_offset == 0);
    assert(chunk->z_offset == 0);
    //assert(chunk->current_node_index == 3);
    //assert(chunk->nodes[2].coord_and_mesh_info.buf[2] != 0);
    //assert(chunk->nodes[0].coord_and_mesh_info.buf[2] != 0);
    //for debugging
    voxel_graph_insert(graph, -9, -32, 12);
    voxel_graph_insert(graph,167, 82 ,-1);

    printf("inserting every node in chunk\n");

    for(int x = 32; x < 48; x++){
        for(int y = 32; y < 48; y++){
            for(int z = 32; z < 48; z++){
                printf("inserting %d %d %d\n", x, y, z);
                voxel_graph_insert(graph, x,y,z);
            }
        }
    }
    printf("checking insertion\n");
    for(int x = 32; x < 48; x++){
        for(int y = 32; y < 48; y++){
            for(int z = 32; z < 48; z++){
                assert(voxel_graph_lookup(graph, x,y,z));
            }
        }
    }
    printf("deleting every node in chunk\n");

    for(int x = 32; x < 48; x++){
        for(int y = 32; y < 48; y++){
            for(int z = 32; z < 48; z++){
                printf("deleting %d %d %d\n", x, y, z);
                voxel_graph_delete(graph, x,y,z);
            }
        }
    }
    printf("checking deletion\n");
    for(int x = 32; x < 48; x++){
        for(int y = 32; y < 48; y++){
            for(int z = 32; z < 48; z++){
                assert(!voxel_graph_lookup(graph, x,y,z));
            }
        }
    }

    voxel_graph_delete(graph, -10, -18, 0);
    voxel_graph_delete(graph, -10, -18, 1);
    voxel_graph_insert(graph, -9, -18, 0);

    //printf("depricated test: neighbour connection and disconnection check:\n");
    //voxel_graph_insert(graph, 1, 1, 2);
    //voxel_graph_insert(graph, 1, 2, 1);
    //voxel_graph_insert(graph, 2, 1, 1);
    //voxel_graph_delete(graph, 1, 1, 1);
    /*
    uint16_t test_coord_center = build_vertex_coords(1, 1, 1);
    uint16_t test_coord_above = build_vertex_coords(1, 1, 2);
    uint16_t test_coord_left = build_vertex_coords(1, 2, 1);
    uint16_t test_coord_ahead = build_vertex_coords(2, 1, 1);

    int64_t test_index_center = chunk_node_lookup(chunk, test_coord_center);
    int64_t test_index_above = chunk_node_lookup(chunk, test_coord_above);
    int64_t test_index_left = chunk_node_lookup(chunk, test_coord_left);
    int64_t test_index_ahead = chunk_node_lookup(chunk, test_coord_ahead);

    assert(chunk->nodes[test_index_center].coord_and_mesh_info.buf[2] == 1);
    assert(!vertex_get_down_bit(&chunk->nodes[test_index_above].coord_and_mesh_info));
    assert(!vertex_get_right_bit(&chunk->nodes[test_index_left].coord_and_mesh_info));
    assert(!vertex_get_back_bit(&chunk->nodes[test_index_ahead].coord_and_mesh_info));
    
    voxel_graph_insert(graph, 1, 1, 1);

    assert(vertex_get_down_bit(&chunk->nodes[test_index_above].coord_and_mesh_info));
    assert(vertex_get_right_bit(&chunk->nodes[test_index_left].coord_and_mesh_info));
    assert(vertex_get_back_bit(&chunk->nodes[test_index_ahead].coord_and_mesh_info));
    
    assert(vertex_get_up_bit(&chunk->nodes[test_index_center].coord_and_mesh_info));
    assert(vertex_get_left_bit(&chunk->nodes[test_index_center].coord_and_mesh_info));
    assert(vertex_get_foward_bit(&chunk->nodes[test_index_center].coord_and_mesh_info));
    assert(!vertex_get_dead_bit(&chunk->nodes[test_index_center].coord_and_mesh_info));

    voxel_graph_insert(graph, 31, 0, 0);
    voxel_graph_insert(graph, 32, 0, 0);
    AltChunk_t* chunk_three = voxel_graph_chunk_hash_table_lookup(graph, 32, 0, 0);
    uint16_t test_chunk_border = build_vertex_coords(31, 0, 0);
    uint16_t test_chunk_border_2 = build_vertex_coords(0, 0, 0);
    int64_t chunk_node_index_2 = chunk_node_lookup(chunk_three, test_chunk_border_2);
    int64_t chunk_node_index_1 = chunk_node_lookup(chunk, test_chunk_border);

    assert(vertex_get_foward_bit(&chunk->nodes[chunk_node_index_1].coord_and_mesh_info));
    assert(vertex_get_back_bit(&chunk_three->nodes[chunk_node_index_2].coord_and_mesh_info));

    voxel_graph_delete(graph, 31, 0, 0);

    assert(!vertex_get_foward_bit(&chunk->nodes[chunk_node_index_1].coord_and_mesh_info));
    assert(!vertex_get_back_bit(&chunk_three->nodes[chunk_node_index_2].coord_and_mesh_info));
    */
    voxel_graph_free(&graph);
    printf("all tests passed\n");
    return 0;
}
