#include "AltGraphNode.h"

AltGraphNode_t alt_graph_node_get_blank(){
    AltGraphNode_t node;
    node.data = 0;
    return node;
}
void alt_graph_node_set(AltGraphNode_t* input, uint32_t position){
    assert(position >= 0 && position <= 3);
    switch(position){
        case 0:
            alt_graph_node_set_first_node_dead(&(*input));
            break;
        case 1:
            alt_graph_node_set_second_node_dead(&(*input));
            break;
        case 2:
            alt_graph_node_set_third_node_dead(&(*input));
            break;
        case 3:
            alt_graph_node_set_fourth_node_dead(&(*input));
            break;
    }
}

bool alt_graph_node_get(AltGraphNode_t* input, uint32_t position){
    assert(position >= 0 && position <= 3);
    bool result = false;
    switch(position){
        case 0:
            result = alt_graph_node_get_first_node_dead(&(*input));
            break;
        case 1:
            result = alt_graph_node_get_second_node_dead(&(*input));
            break;
        case 2:
            result = alt_graph_node_get_third_node_dead(&(*input));
            break;
        case 3:
            result = alt_graph_node_get_fourth_node_dead(&(*input));
            break;
    }
    return result;
}

void alt_graph_node_delete(AltGraphNode_t* input, uint32_t position){
    assert(position >= 0 && position <= 3);
    switch(position){
        case 0:
            alt_graph_node_clear_first_node_dead(&(*input));
            break;
        case 1:
            alt_graph_node_clear_second_node_dead(&(*input));
            break;
        case 2:
            alt_graph_node_clear_third_node_dead(&(*input));
            break;
        case 3:
            alt_graph_node_clear_fourth_node_dead(&(*input));
            break;
    }
}

void alt_graph_node_set_first_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 128;
    input->data = input->data | bit_mask;
}
void alt_graph_node_set_second_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 32;
    input->data = input->data | bit_mask;
}
void alt_graph_node_set_third_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 8;
    input->data = input->data | bit_mask;
}
void alt_graph_node_set_fourth_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 2;
    input->data = input->data | bit_mask;
}
                                                                                
void alt_graph_node_clear_first_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 128;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_clear_second_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 32;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_clear_third_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 8;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_clear_fourth_node_dead(AltGraphNode_t* input){
    uint8_t bit_mask = 2;
    input->data = input->data & ~bit_mask;
}

bool alt_graph_node_get_first_node_dead(AltGraphNode_t* input){
    uint8_t bitmask = 128;
    return input->data & bitmask;
}
bool alt_graph_node_get_second_node_dead(AltGraphNode_t* input){
    uint8_t bitmask = 32;
    return input->data & bitmask;
}
bool alt_graph_node_get_third_node_dead(AltGraphNode_t* input){
    uint8_t bitmask = 8;
    return input->data & bitmask;
}
bool alt_graph_node_get_fourth_node_dead(AltGraphNode_t* input){
    uint8_t bitmask = 2;
    return input->data & bitmask;
}

void alt_graph_node_set_first_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 64;
    input->data = input->data | bit_mask;
}
void alt_graph_node_set_second_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 16;
    input->data = input->data | bit_mask;
}
void alt_graph_node_set_third_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 4;
    input->data = input->data | bit_mask;
}
void alt_graph_node_set_fourth_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 1;
    input->data = input->data | bit_mask;
}
                                                                                
void alt_graph_node_clear_first_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 64;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_clear_second_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 16;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_clear_third_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 4;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_clear_fourth_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 1;
    input->data = input->data & ~bit_mask;
}

bool alt_graph_node_get_first_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 64;
    return input->data & bit_mask;
}
bool alt_graph_node_get_second_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 16;
    return input->data & bit_mask;
}
bool alt_graph_node_get_third_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 4;
    return input->data & bit_mask;
}
bool alt_graph_node_get_fourth_node_visited(AltGraphNode_t* input){
    uint8_t bit_mask = 1;
    return input->data & bit_mask;
}

