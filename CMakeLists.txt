cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 23)

project(lidar-control)

include(cmake/dependencies.cmake)
include(cmake/flags.cmake)

option(RUN_TESTS "Creates and runs unit tests for the project" OFF)

if(RUN_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

include_directories(inc)
file(GLOB SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} liblidar)
add_dependencies(${PROJECT_NAME} libserial)
add_dependencies(${PROJECT_NAME} libclimenu)

target_link_libraries(${PROJECT_NAME}
    lidar
    serial
    climenu
    ${Boost_LIBRARIES}
)
