#include "VoxelHashMap.h"
//helpers
static inline bool point_equals(Point_t* p, int64_t x, int64_t y, int64_t z){
    return p->x == x && p->y == y && p->z == z;
}
static inline int64_t get_next_pow_of_2(uint64_t in){
    if(in == 0){
        return 1;
    }
    in--;
    in = in | (in >> 1);
    in = in | (in >> 2);
    in = in | (in >> 4);
    in = in | (in >> 8);
    in = in | (in >> 16);
    in = in | (in >> 32);
    if(!in){
        return 0;
    }
    return in + 1;
}
static inline PointSlot_t* get_slot(VoxelHashMap_t* hashmap, uint64_t hash){
    hash = hash & (hashmap->capacity - 1);
    return &hashmap->slots[hash];
}
static inline uint8_t attempt_insertion(PointSlot_t* slot, int64_t x, int64_t y, int64_t z){
    switch (slot->state)
    {
    case SLOT_EMPTY:
        slot->state = SLOT_OCCUPIED;
        slot->key.x = x;
        slot->key.y = y;
        slot->key.z = z;
        return 1;
        break;
    case SLOT_OCCUPIED:
        if(!point_equals(&slot->key, x, y, z)){
            return 0;
        }
        return 2;
        break;
    case SLOT_TOMBSTONE:
        if(point_equals(&slot->key, x, y, z)){
            slot->state = SLOT_OCCUPIED;
            return 3;
        }
        return 0;
        break; 
    default:
        return 0;
        break;
    }
}
static inline bool attempt_deletion(PointSlot_t* slot, int64_t x, int64_t y, int64_t z){
    if(!point_equals(&slot->key, x, y, z)){
        return false;
    }
    switch (slot->state)
    {
    case SLOT_EMPTY:
        return false;
        break;
    case SLOT_TOMBSTONE:
        return false;
        break;
    case SLOT_OCCUPIED:
        slot->state = SLOT_TOMBSTONE;
        return true;
        break;
    default:
        return false;
        break;
    }
}

static inline PointSlot_t* voxel_hash_map_insert_with_known_hash(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z, uint64_t known_hash){
    assert(hashmap != NULL);
    uint64_t hash = known_hash;
    PointSlot_t* initial_slot = get_slot(hashmap, hash);
    uint8_t return_code = attempt_insertion(initial_slot, x, y, z);
    if(return_code){
        initial_slot->raw_hash = hash;
        switch (return_code)
        {
        case 1:
            hashmap->occupied_slot_count++;
            /* code */
            break;
        case 2:
            break;
        case 3:
            hashmap->occupied_slot_count++;
            hashmap->tombstome_count--;
            break;
        default:
            break;
        }
        return initial_slot;
    }
    else{
        uint64_t raw_double_hash = build_hash_map_hash(x, y, z, hash);//fibonacci_doublehash(hash);
        for(uint8_t probe_chain_len = 1; probe_chain_len < hashmap->max_probe_chain_len; probe_chain_len++){
            uint64_t double_hash = hash + (raw_double_hash * probe_chain_len);
            PointSlot_t* probe_slot = get_slot(hashmap, double_hash);
            uint8_t return_code_two = attempt_insertion(probe_slot, x, y, z);
            if(return_code_two){
                probe_slot->raw_hash = hash;
                switch (return_code_two)
                {
                case 1:
                    hashmap->occupied_slot_count++;
                    /* code */
                    break;
                case 2:
                    break;
                case 3:
                    hashmap->occupied_slot_count++;
                    hashmap->tombstome_count--;
                    break;
                default:
                    break;
                }
                return probe_slot;
            }
        }
    }
    return NULL;
}

static inline void resize(VoxelHashMap_t* hashmap){
    PointSlot_t* new_array = malloc(sizeof(PointSlot_t) * (hashmap->capacity << 1));
    assert(new_array != NULL);
    if(new_array == NULL){
        return;
    }
    PointSlot_t* old_array = hashmap->slots;
    uint64_t old_capacity = hashmap->capacity;
    hashmap->capacity = hashmap->capacity << 1;
    hashmap->tombstome_count = 0;
    hashmap->occupied_slot_count = 0;
    hashmap->slots = new_array;
    for(uint64_t cnt = 0; cnt < hashmap->capacity; cnt++){
        hashmap->slots[cnt].key.x = 0;
        hashmap->slots[cnt].key.y = 0;
        hashmap->slots[cnt].key.z = 0;
        hashmap->slots[cnt].prev_key.x = 0;
        hashmap->slots[cnt].prev_key.y = 0;
        hashmap->slots[cnt].prev_key.z = 0;
        hashmap->slots[cnt].raw_hash = 0;
        hashmap->slots[cnt].state = SLOT_EMPTY;
        hashmap->slots[cnt].traveled_dist = 999999999.0f;
        hashmap->slots[cnt].astar_heuristic = 0.0f;
        hashmap->slots[cnt].has_prev = 0;
        hashmap->slots[cnt].visited = false;
    }
    for(uint64_t cnt = 0; cnt < old_capacity; cnt++){
        int64_t x = old_array[cnt].key.x;
        int64_t y = old_array[cnt].key.y;
        int64_t z = old_array[cnt].key.z;
        uint64_t hash = old_array[cnt].raw_hash;
        if(old_array[cnt].state == SLOT_OCCUPIED){
            PointSlot_t* new_slot = voxel_hash_map_insert_with_known_hash(hashmap, x, y, z, hash);
            assert(new_slot != NULL);
            new_slot->prev_key = old_array[cnt].prev_key;
            new_slot->astar_heuristic = old_array[cnt].astar_heuristic;
            new_slot->traveled_dist = old_array[cnt].traveled_dist;
            new_slot->has_prev = old_array[cnt].has_prev;
            new_slot->visited = old_array[cnt].visited;
        }
    }
    free(old_array);
}
//end helpers
VoxelHashMap_t* voxel_hash_map_init(uint64_t initial_capacity, uint8_t probe_chain_limit, float max_load_factor){
    VoxelHashMap_t* hashmap = malloc(sizeof(VoxelHashMap_t));
    hashmap->max_probe_chain_len = probe_chain_limit;
    hashmap->load_factor_threshold = max_load_factor;
    hashmap->occupied_slot_count = 0;
    hashmap->tombstome_count = 0;
    hashmap->capacity = get_next_pow_of_2(initial_capacity);
    hashmap->slots = malloc(sizeof(PointSlot_t) * hashmap->capacity);
    srand(time(NULL));
    hashmap->hash_seed = rand();
    for(int64_t cnt = 0; cnt < hashmap->capacity; cnt++){
        hashmap->slots[cnt].key.x = 0;
        hashmap->slots[cnt].key.y = 0;
        hashmap->slots[cnt].key.z = 0;
        hashmap->slots[cnt].prev_key.x = 0;
        hashmap->slots[cnt].prev_key.y = 0;
        hashmap->slots[cnt].prev_key.z = 0;
        hashmap->slots[cnt].raw_hash = 0;
        hashmap->slots[cnt].state = SLOT_EMPTY;
        hashmap->slots[cnt].visited = false;
        hashmap->slots[cnt].traveled_dist = 999999999.0f;
        hashmap->slots[cnt].astar_heuristic = 0.0f;
        hashmap->slots[cnt].has_prev = 0;
    }
    return hashmap;
}

void voxel_hash_map_free(VoxelHashMap_t* hashmap){
    free(hashmap->slots);
    free(hashmap);
}
PointSlot_t* voxel_hash_map_insert(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z){
    int8_t resizes = 0;
    while(resizes < 5){
        assert(hashmap != NULL);
        int64_t total_slot_count = hashmap->occupied_slot_count + hashmap->tombstome_count;
        float load_factor = (float)total_slot_count / (float)hashmap->capacity;
        if(load_factor >= hashmap->load_factor_threshold){
            resize(hashmap);
        }
        uint64_t hash = build_hash_map_hash(x, y, z, hashmap->hash_seed);//build_fibonacci_hash_from_coords(x, y, z);
        PointSlot_t* initial_slot = get_slot(hashmap, hash);
        uint8_t return_code = attempt_insertion(initial_slot, x, y, z);
        if(return_code){
            initial_slot->raw_hash = hash;
            switch (return_code)
            {
            case 1:
                hashmap->occupied_slot_count++;
                /* code */
                break;
            case 2:
                break;
            case 3:
                hashmap->occupied_slot_count++;
                hashmap->tombstome_count--;
                break;
            default:
                break;
            }
            return initial_slot;
        }
        else{
            uint64_t raw_double_hash = build_hash_map_hash(x, y, z, hash);//fibonacci_doublehash(hash);
            for(uint8_t probe_chain_len = 1; probe_chain_len < hashmap->max_probe_chain_len; probe_chain_len++){
                uint64_t double_hash = hash + (raw_double_hash * probe_chain_len);
                PointSlot_t* probe_slot = get_slot(hashmap, double_hash);
                uint8_t return_code_two = attempt_insertion(probe_slot, x, y, z);
                if(return_code_two){
                    probe_slot->raw_hash = hash;
                    switch (return_code_two)
                    {
                    case 1:
                        hashmap->occupied_slot_count++;
                        /* code */
                        break;
                    case 2:
                        break;
                    case 3:
                        hashmap->occupied_slot_count++;
                        hashmap->tombstome_count--;
                        break;
                    default:
                        break;
                    }

                    return probe_slot;
                }
            }
        }
        resize(hashmap); // this part of the code only triggers when the probe chain length is longer than 10 
        resizes++;
    }
    return NULL;
}
PointSlot_t* voxel_hash_map_lookup(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z){
    assert(hashmap != NULL);
    uint64_t hash = build_hash_map_hash(x, y, z, hashmap->hash_seed);//build_fibonacci_hash_from_coords(x, y, z);
    PointSlot_t* initial_slot = get_slot(hashmap, hash);
    if(initial_slot->state == SLOT_EMPTY){
        return NULL;
    }
    if(point_equals(&initial_slot->key, x, y, z)){
        if(initial_slot->state == SLOT_TOMBSTONE){
            return NULL;
        }
        return initial_slot;
    }
    else{
        uint64_t raw_double_hash = build_hash_map_hash(x, y, z, hash);//fibonacci_doublehash(hash);
        for(uint8_t probe_chain_len = 1; probe_chain_len < hashmap->max_probe_chain_len; probe_chain_len++){
            uint64_t double_hash = hash + (raw_double_hash * probe_chain_len);
            PointSlot_t* probe_slot = get_slot(hashmap, double_hash);
            if(probe_slot->state == SLOT_EMPTY){
                return NULL;
            }
            if(point_equals(&probe_slot->key, x, y, z)){
                if(probe_slot->state == SLOT_TOMBSTONE){
                    return NULL;
                }
                return probe_slot;
            }
        }
    }
    return NULL;
}
bool voxel_hash_map_remove(VoxelHashMap_t* hashmap, int64_t x, int64_t y, int64_t z){
    assert(hashmap != NULL);
    uint64_t hash = build_hash_map_hash(x, y, z, hashmap->hash_seed);//build_fibonacci_hash_from_coords(x, y, z);
    PointSlot_t* initial_slot = get_slot(hashmap, hash);
    if(initial_slot->state == SLOT_EMPTY){
        return false;
    }
    if(attempt_deletion(initial_slot, x, y, z)){
        initial_slot->raw_hash = hash;
        hashmap->occupied_slot_count--;
        hashmap->tombstome_count++;
        return true;
    }
    else{
        uint64_t raw_double_hash = build_hash_map_hash(x, y, z, hash);//fibonacci_doublehash(hash);
        for(uint8_t probe_chain_len = 1; probe_chain_len < hashmap->max_probe_chain_len; probe_chain_len++){
            uint64_t double_hash = hash + (raw_double_hash * probe_chain_len);
            PointSlot_t* probe_slot = get_slot(hashmap, double_hash);
            if(attempt_deletion(probe_slot, x, y, z)){
                probe_slot->raw_hash = hash;
                hashmap->occupied_slot_count--;
                hashmap->tombstome_count++;
                return true;
            }
        }
    }
    return false;
}