cmake_minimum_required(VERSION 2.8.3)
project(dumbo_arm_test)

find_package(catkin REQUIRED COMPONENTS kvaser_canlib dumbo_powercube_chain)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS kvaser_canlib dumbo_powercube_chain
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/dumbo_arm_test.c)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)