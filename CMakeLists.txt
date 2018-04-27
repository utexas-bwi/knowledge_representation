cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        std_msgs
        sensor_msgs
        )

link_directories(
        ${catkin_LINK_DIRS}
)

add_service_files(
        FILES
        AddObject.srv
        RemoveObject.srv
        ResolveObjectCorrespondences.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

find_package(Eigen3 REQUIRED)

find_library(mysql NAMES libmysqlcppconn8.so)

catkin_package(
        INCLUDE_DIRS include /usr/local/mysql/connector-c++-8.0/include
        LIBRARIES libknowledge_rep ${mysql}
        CATKIN_DEPENDS pcl_ros pcl_conversions
        roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        /usr/local/mysql/connector-c++-8.0/include
        ${Eigen3_INCLUDE_DIRS}
)


add_library(libknowledge_rep src/libknowledge_rep/LongTermMemoryConduit.cpp src/libknowledge_rep/MemoryConduit.cpp src/libknowledge_rep/ShortTermMemory.cpp)
target_link_libraries(libknowledge_rep ${mysql} ${catkin_LIBRARIES})

add_executable(test_ltmc src/test_ltmc.cpp)
add_dependencies(test_ltmc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_ltmc libknowledge_rep ${catkin_LIBRARIES})

add_executable(test_stmc src/test_stmc.cpp)
add_dependencies(test_stmc ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(test_stmc libknowledge_rep ${catkin_LIBRARIES})


add_executable(initialize_planner_test_configuration src/initialize_planner_test_configuration.cpp)
add_dependencies(initialize_planner_test_configuration ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(initialize_planner_test_configuration libknowledge_rep ${catkin_LIBRARIES})

add_executable(short_term_memory_node src/short_term_memory_node.cpp src/short_term_memory_node.cpp)
add_dependencies(short_term_memory_node ${knowledge_representation_EXPORTED_TARGETS})
target_link_libraries(short_term_memory_node libknowledge_rep ${catkin_LIBRARIES})


install(TARGETS libknowledge_rep
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(TARGETS
        initialize_planner_test_configuration
        test_ltmc
        short_term_memory_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
