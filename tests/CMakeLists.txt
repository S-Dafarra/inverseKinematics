cmake_minimum_required(VERSION 2.8.12)

# set the project name
project(InverseKinematicsTest)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/data)

add_executable(${PROJECT_NAME} testIK.cpp)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/data/URDFdir.h.in" "${CMAKE_CURRENT_BINARY_DIR}/data/URDFdir.h" @ONLY)

target_link_libraries(${PROJECT_NAME} inverseKinematics ${iDynTree_LIBRARIES})

add_test(NAME testIK
        COMMAND ${PROJECT_NAME})