add_library(dvg STATIC
    src/Dvg.c
    src/DynamicObjectRemoval.c
    src/Chunk.c
    src/ChunkBitmap.c
    src/DvgVector.c
    src/FibonacciHash.c
    src/HashUtils.c
    src/murmur3.c
    src/VoxelHashMap.c
    src/VoxelPriorityQueue.c
    src/SphericalDedup.c
)

target_include_directories(dvg
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(UNIX)
    target_link_libraries(dvg PUBLIC m)
endif()