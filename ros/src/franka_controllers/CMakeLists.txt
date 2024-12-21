cmake_minimum_required(VERSION 3.2.0)
project(franka_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  franka_hw
  franka_control
  hardware_interface
  pluginlib
  geometry_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

include(ExternalProject)
ExternalProject_Add(LCQPow
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow
  BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/build
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR>
  INSTALL_COMMAND ""
)

add_library(qpOASES_lib SHARED IMPORTED)
set_target_properties(qpOASES_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/build/lib/${CMAKE_FIND_LIBRARY_PREFIXES}qpOASES${CMAKE_SHARED_LIBRARY_SUFFIX}
  IMPORTED_NO_SONAME TRUE
)
set(qpoases_include ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/build/external/src/qpoases/include)

add_library(osqp_lib SHARED IMPORTED)
set_target_properties(osqp_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/build/lib/${CMAKE_FIND_LIBRARY_PREFIXES}osqp${CMAKE_SHARED_LIBRARY_SUFFIX}
)
set(osqp_include ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/build/external/src/osqp/include)

add_library(LCQPow_lib SHARED IMPORTED)
set_target_properties(LCQPow_lib PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/build/lib/${CMAKE_FIND_LIBRARY_PREFIXES}lcqpow${CMAKE_SHARED_LIBRARY_SUFFIX}
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/submodule/LCQPow/include
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_controllers
  CATKIN_DEPENDS 
    controller_interface
    franka_hw
    franka_control
    hardware_interface
    pluginlib
    geometry_msgs
    visualization_msgs
    pal_statistics
  #DEPENDS 
    #EIGEN3
    #Franka
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  ${qpoases_include}
  ${osqp_include}
)

add_library(${PROJECT_NAME} SHARED
  src/joint_position_controller.cpp
  src/LCQP_controller_frankaModel.cpp
  src/LCQP_controller_multipleObstacle.cpp
  src/LCQP_controller_quickconcept.cpp
  src/LCQP_controller.cpp
  src/FrankaModel.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${Franka_LIBRARIES}
  LCQPow_lib
  qpOASES_lib
  osqp_lib
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_franka_controllers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
