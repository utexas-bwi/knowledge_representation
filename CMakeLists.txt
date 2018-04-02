cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )


include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        /usr/local/mysql/connector-c++-8.0/include
)

link_directories(
        ${catkin_LINK_DIRS}

)

catkin_package(
        CATKIN_DEPENDS
        roscpp
)

find_library(mysql NAMES libmysqlcppconn8.so)
add_library(libworld_model src/libworld_model/LongTermMemoryConduit.cpp src/libworld_model/MemoryConduit.cpp include/villa_world_model/MemoryConduit.h)
target_link_libraries(libworld_model ${mysql} ${catkin_LIBRARIES})

add_executable(world_model_tester src/test_world_model.cpp)
target_link_libraries(world_model_tester libworld_model ${catkin_LIBRARIES})

add_executable(short_term_memory_node src/short_term_memory_node.cpp src/short_term_memory_node.cpp)
target_link_libraries(short_term_memory_node libworld_model ${catkin_LIBRARIES})