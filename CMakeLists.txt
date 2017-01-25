cmake_minimum_required(VERSION 2.8.12)

project(inverseKinematics)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

find_package(iDynTree REQUIRED)
find_package(IPOPT REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include_directories(include)

add_library(${PROJECT_NAME}  SHARED src/InverseKinematicsIPOPT.cpp include/InverseKinematicsIPOPT.h
                                    src/InverseKinematics.cpp include/InverseKinematics.h)

target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})

include(CTest)
if(BUILD_TESTING) 
   add_subdirectory(tests)
endif()