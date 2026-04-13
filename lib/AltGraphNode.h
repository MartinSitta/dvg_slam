#ifndef ALTGRAPHNODE_H                                                             
#define ALTGRAPHNODE_H                                                             
                                                                                
#ifdef __cplusplus                                                              
extern "C"{                                                                     
#endif         

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
                                                                                
typedef struct{
    uint8_t data;
}AltGraphNode_t; 
AltGraphNode_t alt_graph_node_get_blank();

void alt_graph_node_set(AltGraphNode_t* input, uint32_t position);
bool alt_graph_node_get(AltGraphNode_t* input, uint32_t position);
void alt_graph_node_delete(AltGraphNode_t* input, uint32_t position);

void alt_graph_node_set_first_node_dead(AltGraphNode_t* input);
void alt_graph_node_set_second_node_dead(AltGraphNode_t* input);
void alt_graph_node_set_third_node_dead(AltGraphNode_t* input);
void alt_graph_node_set_fourth_node_dead(AltGraphNode_t* input);
                                                                                
void alt_graph_node_clear_first_node_dead(AltGraphNode_t* input);
void alt_graph_node_clear_second_node_dead(AltGraphNode_t* input);
void alt_graph_node_clear_third_node_dead(AltGraphNode_t* input);
void alt_graph_node_clear_fourth_node_dead(AltGraphNode_t* input);

bool alt_graph_node_get_first_node_dead(AltGraphNode_t* input);
bool alt_graph_node_get_second_node_dead(AltGraphNode_t* input);
bool alt_graph_node_get_third_node_dead(AltGraphNode_t* input);
bool alt_graph_node_get_fourth_node_dead(AltGraphNode_t* input);

void alt_graph_node_set_first_node_visited(AltGraphNode_t* input);
void alt_graph_node_set_second_node_visited(AltGraphNode_t* input);
void alt_graph_node_set_third_node_visited(AltGraphNode_t* input);
void alt_graph_node_set_fourth_node_visited(AltGraphNode_t* input);
                                                                                
void alt_graph_node_clear_first_node_visited(AltGraphNode_t* input);
void alt_graph_node_clear_second_node_visited(AltGraphNode_t* input);
void alt_graph_node_clear_third_node_visited(AltGraphNode_t* input);
void alt_graph_node_clear_fourth_node_visited(AltGraphNode_t* input);

bool alt_graph_node_get_first_node_visited(AltGraphNode_t* input);
bool alt_graph_node_get_second_node_visited(AltGraphNode_t* input);
bool alt_graph_node_get_third_node_visited(AltGraphNode_t* input);
bool alt_graph_node_get_fourth_node_visited(AltGraphNode_t* input);
 
#ifdef __cplusplus                                                              
}                                                                               
#endif                                                                          
                                                                                
#endif                                                                          
                  
