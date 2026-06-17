#include "VoxelPriorityQueue.h"
#include "VoxelHashMap.h"
//NOTE: THE IMPLEMENTATION IS PRETTY SIMILAR TO THE IMPLEMENTATION OF GEEKSFORGEEKS
//KEY DIFFERENCE IS I PLAN TO RESIZE THE ARRAY AS I GO
//helpers
static inline float get_cost(VoxelHashMap_t* nodes, Point_t* key){
    PointSlot_t* slot = voxel_hash_map_lookup(nodes, key->x, key->y, key->z);
    if(slot == NULL) return 999999999.0f;
    return slot->traveled_dist + slot->astar_heuristic;
}
static inline void swap(Point_t* heap, int64_t index_1, int64_t index_2){
    Point_t temp = heap[index_2];
    heap[index_2] = heap[index_1];
    heap[index_1] = temp;
}
static inline void heapify_up(Point_t* heap, int64_t index, VoxelHashMap_t* nodes){
    if(index && get_cost(nodes, &heap[(index-1)/2]) > get_cost(nodes, &heap[index])){
        swap(heap, (index-1)/2, index);
        heapify_up(heap, (index-1)/2, nodes);
    }
}
static inline void heapify_down(Point_t* heap, int64_t index, 
                                int64_t capacity, VoxelHashMap_t* nodes){
    int64_t smallest = index;
    int64_t left  = 2*index+1;
    int64_t right = 2*index+2;
    if(left  < capacity && get_cost(nodes, &heap[left])  < get_cost(nodes, &heap[smallest])){
        smallest = left;
    }
    if(right < capacity && get_cost(nodes, &heap[right]) < get_cost(nodes, &heap[smallest])){
        smallest = right;
    }
    if(smallest != index){
        swap(heap, index, smallest);
        heapify_down(heap, smallest, capacity, nodes);
    }
}

void resize(VoxelPriorityQueue_t* queue){
    Point_t* new_array = malloc(sizeof(Point_t) * queue->capacity * 2);
    assert(new_array != NULL);
    for(int64_t cnt = 0; cnt < queue->capacity; cnt++){
        new_array[cnt] = queue->array[cnt];
    }
    for(int64_t cnt = queue->capacity; cnt < queue->capacity * 2; cnt++){
        new_array[cnt].x = 0;
        new_array[cnt].y = 0;
        new_array[cnt].z = 0;
    }
    queue->capacity = queue->capacity * 2;
    Point_t* old_array = queue->array;
    queue->array = new_array;
    free(old_array);
}




//end helpers

VoxelPriorityQueue_t* voxel_priority_queue_init(int64_t capacity){
    VoxelPriorityQueue_t* output = malloc(sizeof(VoxelPriorityQueue_t));
    output->capacity = capacity;
    output->current_element = 0;
    output->array = malloc(sizeof(Point_t) * output->capacity);
    if(output->array == NULL){
        return NULL;
    }
    for(int64_t cnt = 0; cnt < capacity; cnt++){
        output->array[cnt].x = 0;
        output->array[cnt].y = 0;
        output->array[cnt].z = 0;
    }
    return output;
}

void voxel_priority_queue_free(VoxelPriorityQueue_t* queue){
    free(queue->array);
    free(queue);
}

void voxel_priority_queue_enqueue(VoxelPriorityQueue_t* queue, 
                                    Point_t key,
                                    VoxelHashMap_t* nodes){
    PointSlot_t* slot = voxel_hash_map_lookup(nodes, key.x, key.y, key.z);
    if(slot == NULL) return;
    if(queue->current_element >= queue->capacity) resize(queue);
    queue->array[queue->current_element] = key;
    heapify_up(queue->array, queue->current_element, nodes);
    queue->current_element++;
}
DequeueRetObject_t voxel_priority_queue_dequeue(VoxelPriorityQueue_t* queue,
                                                VoxelHashMap_t* nodes){
    DequeueRetObject_t ret;
    Point_t ret_init_point = {0,0,0};
    ret.point = ret_init_point;
    ret.valid = false;
    if(queue->current_element == 0) return ret;
    Point_t output_key = queue->array[0]; // return pointer to key in queue
    PointSlot_t* slot = voxel_hash_map_lookup(nodes, output_key.x, 
                                               output_key.y, output_key.z);
    queue->current_element--;
    queue->array[0] = queue->array[queue->current_element];
    heapify_down(queue->array, 0, queue->current_element, nodes);
    ret.point = output_key;
    ret.valid = true;
    return ret;
}
Point_t* voxel_priority_queue_peek(VoxelPriorityQueue_t* queue){
    if(queue->current_element == 0) return NULL;
    return &queue->array[0];
}