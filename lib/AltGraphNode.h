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

void alt_graph_node_first_set_bit_one(AltGraphNode_t* input);
void alt_graph_node_second_set_bit_one(AltGraphNode_t* input);
void alt_graph_node_third_set_bit_one(AltGraphNode_t* input);
void alt_graph_node_fourth_set_bit_one(AltGraphNode_t* input);
                                                                                
void alt_graph_node_first_clear_bit_one(AltGraphNode_t* input);
void alt_graph_node_second_clear_bit_one(AltGraphNode_t* input);
void alt_graph_node_third_clear_bit_one(AltGraphNode_t* input);
void alt_graph_node_fourth_clear_bit_one(AltGraphNode_t* input);

bool alt_graph_node_first_get_bit_one(AltGraphNode_t* input);
bool alt_graph_node_second_get_bit_one(AltGraphNode_t* input);
bool alt_graph_node_third_get_bit_one(AltGraphNode_t* input);
bool alt_graph_node_fourth_get_bit_one(AltGraphNode_t* input);

void alt_graph_node_first_set_bit_two(AltGraphNode_t* input);
void alt_graph_node_second_set_bit_two(AltGraphNode_t* input);
void alt_graph_node_third_set_bit_two(AltGraphNode_t* input);
void alt_graph_node_fourth_set_bit_two(AltGraphNode_t* input);
                                                                                
void alt_graph_node_first_clear_bit_two(AltGraphNode_t* input);
void alt_graph_node_second_clear_bit_two(AltGraphNode_t* input);
void alt_graph_node_third_clear_bit_two(AltGraphNode_t* input);
void alt_graph_node_fourth_clear_bit_two(AltGraphNode_t* input);

bool alt_graph_node_first_get_bit_two(AltGraphNode_t* input);
bool alt_graph_node_second_get_bit_two(AltGraphNode_t* input);
bool alt_graph_node_third_get_bit_two(AltGraphNode_t* input);
bool alt_graph_node_fourth_get_bit_two(AltGraphNode_t* input);
 
#ifdef __cplusplus                                                              
}                                                                               
#endif                                                                          
                                                                                
#endif                                                                          
                  
