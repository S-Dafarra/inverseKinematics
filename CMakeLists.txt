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
                                    src/InverseKinematicsV2IPOPT.cpp include/InverseKinematicsV2IPOPT.h
                                    src/InverseKinematics.cpp include/InverseKinematics.h)

target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER include/InverseKinematics.h)

include(CTest)
if(BUILD_TESTING) 
   add_subdirectory(tests)
endif()

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/InverseKinematics/
        )
