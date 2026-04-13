#include "GraphNode.h"
#include <assert.h>

GraphNode_t graph_node_init(){
    GraphNode_t output;
    output.coord_and_mesh_info.buf[0] = 0;
    output.coord_and_mesh_info.buf[1] = 0;
    output.coord_and_mesh_info.buf[2] = 0;

    /*
    output.upper_neighbour = -1;
    output.lower_neighbour = -1;
    output.left_neighbour = -1;
    output.right_neighbour = -1;
    output.foward_neighbour = -1;
    output.back_neighbour = -1;
    */
    assert(output.coord_and_mesh_info.buf[0] == 0);
    assert(output.coord_and_mesh_info.buf[1] == 0);
    assert(output.coord_and_mesh_info.buf[2] == 0);
    //assert(output.upper_neighbour == -1);
    //assert(output.back_neighbour == -1);
    
    return output;
}
