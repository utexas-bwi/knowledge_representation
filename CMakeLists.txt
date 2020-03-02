cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)

set(CMAKE_CXX_STANDARD 11)

find_package(BISON)
find_package(FLEX)

find_package(catkin REQUIRED COMPONENTS
        roslint
        )
find_package(Boost REQUIRED COMPONENTS python)

find_package(PythonLibs 2.7 REQUIRED)

find_package(PostgreSQL)

find_library(libpqxx NAMES pqxx)

find_library(mysql NAMES libmysqlcppconn8.so)


# Check to see if we have the right version of MySQL Cpp Connector installed
if(POSTGRESQL_FOUND OR libpqxx)

    set(POSTGRES_AVAILABLE true)
    set(DB_INCLUDES ${PostgreSQL_INCLUDE_DIRS})
    set(EXTRA_EXPORTED_INCLUDES "")
    set(DB_LIBS ${PostgreSQL_LIBRARIES} ${libpqxx})
    add_definitions(-DUSE_POSTGRESQL)
elseif(mysql)
    message(WARNING "PostgreSQL not found. Using incomplete MySQL implementation.")
    set(MYSQL_AVAILABLE true)
    set(DB_INCLUDES /usr/local/mysql/connector-c++-8.0/include)
    set(EXTRA_EXPORTED_INCLUDES ${DB_INCLUDES})
    set(DB_LIBS ${mysql})
    add_definitions(-DUSE_MYSQL)
else()
    message(FATAL_ERROR "Neither MySQL nor PostgreSQL found. Please install one.")
endif()

# Trick catkin lint into being okay with MySQL include exports
if(POSTGRESQL_FOUND OR libpqxx)
    set(EXTRA_EXPORTED_INCLUDES "")
endif()
catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include ${EXTRA_EXPORTED_INCLUDES}
        LIBRARIES knowledge_rep ${DB_LIBS}
        CATKIN_DEPENDS
)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${DB_INCLUDES}
        include
        ${PYTHON_INCLUDE_DIRS}
)


if ("${MYSQL_AVAILABLE}" OR "${POSTGRES_AVAILABLE}")


#BISON_TARGET(parser src/libknowledge_rep/parser.yy ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Parser.cpp COMPILE_FLAGS --defines=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Parser.h)
#FLEX_TARGET(scanner src/libknowledge_rep/lexer.l  ${CMAKE_CURRENT_BINARY_DIR}/Lexer.cpp)


#ADD_FLEX_BISON_DEPENDENCY(scanner parser)
if (MYSQL_AVAILABLE)
    set(DB_SOURCES src/libknowledge_rep/mysql/LongTermMemoryConduitMySQL.cpp)
    # Symlink the LTMC header into place. We have to put this header in the public
    # interface in order to instantiate the template of the base class. This class
    # will not be used alone, so methods in its interface can't really be used.
    add_custom_target(link_target ALL
            COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/src/libknowledge_rep/mysql/LongTermMemoryConduitMySQL.h ${PROJECT_SOURCE_DIR}/include/knowledge_representation/LongTermMemoryConduit.h)
elseif (POSTGRES_AVAILABLE)
    set(DB_SOURCES src/libknowledge_rep/postgresql/LongTermMemoryConduitPostgreSQL.cpp)
    add_custom_target(link_target ALL
            COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/src/libknowledge_rep/postgresql/LongTermMemoryConduitPostgreSQL.h ${PROJECT_SOURCE_DIR}/include/knowledge_representation/LongTermMemoryConduit.h)
endif()
add_library(knowledge_rep
        ${DB_SOURCES}
        src/libknowledge_rep/convenience.cpp
       # src/libknowledge_rep/Compiler.cpp
       # ${BISON_parser_OUTPUTS}
       # ${FLEX_scanner_OUTPUTS}
        )

add_dependencies(knowledge_rep link_target)
target_link_libraries(knowledge_rep ${DB_LIBS} ${catkin_LIBRARIES})

add_library(_libknowledge_rep_wrapper_cpp src/libknowledge_rep/python_wrapper.cpp)
target_link_libraries(_libknowledge_rep_wrapper_cpp
        knowledge_rep
        ${DB_LIBS}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(show_me src/show_me.cpp)
target_link_libraries(show_me knowledge_rep ${catkin_LIBRARIES})


set_target_properties(_libknowledge_rep_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

catkin_add_env_hooks(${PROJECT_NAME} SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(TARGETS show_me
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS knowledge_rep
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS _libknowledge_rep_wrapper_cpp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

file(GLOB_RECURSE ${PROJECT_NAME}_CPP_SRC
        RELATIVE ${PROJECT_SOURCE_DIR} src/lib${PROJECT_NAME}/*.cpp src/libknowledge_rep/*.h include/${PROJECT_NAME}/*.h test/*.cpp)
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-build/header_guard,-runtime/references,-build/c++11,-whitespace/braces")
roslint_cpp(${${PROJECT_NAME}_CPP_SRC})

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
        RELATIVE ${PROJECT_SOURCE_DIR} scripts/populate_* scripts/test_ltmc scripts/dump_to_owl)

# List FILTER isn't supported in 16.04's CMake
# list(FILTER ${PROJECT_NAME}_PY_SCRIPTS EXCLUDE REGEX ".*.sh")
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRC
        RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/*.py)
set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS})
roslint_python(${${PROJECT_NAME}_PY_SRC})

### TEST TARGETS
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_ltmc test/ltmc.cpp test/map_types.cpp)
    target_link_libraries(test_ltmc knowledge_rep ${catkin_LIBRARIES})

    catkin_add_nosetests(test/ltmc.py)

#    catkin_add_gtest(test_parser test/parser.cpp)
#    target_link_libraries(test_parser knowledge_rep ${catkin_LIBRARIES})
endif()


endif ()
