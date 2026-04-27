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
            alt_graph_node_first_set_bit_one(&(*input));
            break;
        case 1:
            alt_graph_node_second_set_bit_one(&(*input));
            break;
        case 2:
            alt_graph_node_third_set_bit_one(&(*input));
            break;
        case 3:
            alt_graph_node_fourth_set_bit_one(&(*input));
            break;
    }
}

bool alt_graph_node_get(AltGraphNode_t* input, uint32_t position){
    assert(position >= 0 && position <= 3);
    bool result = false;
    switch(position){
        case 0:
            result = alt_graph_node_first_get_bit_one(&(*input));
            break;
        case 1:
            result = alt_graph_node_second_get_bit_one(&(*input));
            break;
        case 2:
            result = alt_graph_node_third_get_bit_one(&(*input));
            break;
        case 3:
            result = alt_graph_node_fourth_get_bit_one(&(*input));
            break;
    }
    return result;
}

void alt_graph_node_delete(AltGraphNode_t* input, uint32_t position){
    assert(position >= 0 && position <= 3);
    switch(position){
        case 0:
            alt_graph_node_first_clear_bit_one(&(*input));
            break;
        case 1:
            alt_graph_node_second_clear_bit_one(&(*input));
            break;
        case 2:
            alt_graph_node_third_clear_bit_one(&(*input));
            break;
        case 3:
            alt_graph_node_fourth_clear_bit_one(&(*input));
            break;
    }
}

void alt_graph_node_first_set_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 128;
    input->data = input->data | bit_mask;
}
void alt_graph_node_second_set_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 32;
    input->data = input->data | bit_mask;
}
void alt_graph_node_third_set_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 8;
    input->data = input->data | bit_mask;
}
void alt_graph_node_fourth_set_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 2;
    input->data = input->data | bit_mask;
}
                                                                                
void alt_graph_node_first_clear_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 128;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_second_clear_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 32;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_third_clear_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 8;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_fourth_clear_bit_one(AltGraphNode_t* input){
    uint8_t bit_mask = 2;
    input->data = input->data & ~bit_mask;
}

bool alt_graph_node_first_get_bit_one(AltGraphNode_t* input){
    uint8_t bitmask = 128;
    return input->data & bitmask;
}
bool alt_graph_node_second_get_bit_one(AltGraphNode_t* input){
    uint8_t bitmask = 32;
    return input->data & bitmask;
}
bool alt_graph_node_third_get_bit_one(AltGraphNode_t* input){
    uint8_t bitmask = 8;
    return input->data & bitmask;
}
bool alt_graph_node_fourth_get_bit_one(AltGraphNode_t* input){
    uint8_t bitmask = 2;
    return input->data & bitmask;
}

void alt_graph_node_first_set_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 64;
    input->data = input->data | bit_mask;
}
void alt_graph_node_second_set_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 16;
    input->data = input->data | bit_mask;
}
void alt_graph_node_third_set_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 4;
    input->data = input->data | bit_mask;
}
void alt_graph_node_fourth_set_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 1;
    input->data = input->data | bit_mask;
}
                                                                                
void alt_graph_node_first_clear_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 64;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_second_clear_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 16;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_third_clear_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 4;
    input->data = input->data & ~bit_mask;
}
void alt_graph_node_fourth_clear_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 1;
    input->data = input->data & ~bit_mask;
}

bool alt_graph_node_first_get_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 64;
    return input->data & bit_mask;
}
bool alt_graph_node_second_get_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 16;
    return input->data & bit_mask;
}
bool alt_graph_node_third_get_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 4;
    return input->data & bit_mask;
}
bool alt_graph_node_fourth_get_bit_two(AltGraphNode_t* input){
    uint8_t bit_mask = 1;
    return input->data & bit_mask;
}

