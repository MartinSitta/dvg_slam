#ifndef GRAPHNODE_H
#define GRAPHNODE_H

#ifdef __cplusplus
extern "C"{
#endif

#include "GraphDefines.h"
#include "Vertex.h"

typedef struct{
    Vertex_t coord_and_mesh_info;
    //chunk_node_array_entry_t upper_neighbour;
    //chunk_node_array_entry_t lower_neighbour;
    //chunk_node_array_entry_t left_neighbour;
    //chunk_node_array_entry_t right_neighbour;
    //chunk_node_array_entry_t foward_neighbour;
    //chunk_node_array_entry_t back_neighbour;
}GraphNode_t;
GraphNode_t graph_node_init();


#ifdef __cplusplus
}
#endif

#endif


