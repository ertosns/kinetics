cmake_minimum_required(VERSION 3.2)

set(project_name kinetics)

project(${project_name})

# debuging mode
#set(CMAKE_BUILD_TYPE Debug)

include_directories(include /usr/include/eigen3)
include_directories(include /usr/include/gtest)
find_library(gtest gtest)

set(test_name kinetics_test)
find_library(pthread pthread)
find_library(algebra algebra)


#set c++17 using filesystem header
set(CMAKE_CXX_STANDARD 17)


# add algebra library

########################
# build libkinetics.a
########################
add_library(${project_name} STATIC
  ./src/kinetics.cpp
  ./src/kinematics.cpp
  ./src/trajectory.cpp
  ./include/kinetics.hpp
  ./include/kinematics.hpp
  ./include/trajectory.hpp
  ./include/mission_planning.hpp
  )

################
# build tests
################
add_executable(${test_name} ./tests/kinematics_test.cpp
  ./tests/kinetics_test.cpp
  ./tests/trajectory_test.cpp
  ./tests/mission_planning_test.cpp
  ./src/kinematics.cpp
  ./src/kinetics.cpp
  ./src/trajectory.cpp
  ./include/kinematics.hpp
  ./include/kinetics.hpp
  ./include/trajectory.hpp
  ./include/mission_planning.hpp
  )
target_link_libraries(${test_name} gtest pthread algebra)

################
# build docs
################
execute_process(COMMAND doxygen docs/Doxyfile
  WORKING_DIRECTORY docs
  )
execute_process(COMMAND make
  WORKING_DIRECTORY docs/latex
  )

install(TARGETS ${project_name})
install(FILES include/astar.hpp
  include/kinematics.hpp
  include/logger.hpp
  include/mission.hpp
  include/obstacles.hpp
  include/trajectory.hpp
  include/graph.hpp
  include/kinetics.hpp
  include/manipulation.hpp
  include/mission_planning.hpp
  include/rrt.hpp
  DESTINATION /usr/include/${project_name})
