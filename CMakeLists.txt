cmake_minimum_required(VERSION 2.8.12)

project(inverseKinematics)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

find_package(iDynTree REQUIRED)
find_package(IPOPT REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(inverseKinematics inverseKinematics.cpp)

target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})


