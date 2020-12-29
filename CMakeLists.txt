cmake_minimum_required(VERSION 3.2)

set(project_name kinetics)

project(${project_name})

# debuging mode
#set(CMAKE_BUILD_TYPE Debug)

#TODO  download algebra from github.com/ertosns/algebra if it isn't present.
add_subdirectory(algebra)

include_directories(include /usr/include/eigen3)
include_directories(include /usr/include/gtest)
find_library(gtest gtest)

set(test_name kinetics_test)
find_library(pthread pthread)

###########################
# include algebra header
include_directories(include ./algebra/source)
# add algebra library
find_library(algebra algebra
  PATHS ./build/algebra
  )

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
target_link_libraries(${test_name} gtest pthread)

################
# build docs
################
execute_process(COMMAND doxygen docs/Doxyfile
  WORKING_DIRECTORY docs
  )
execute_process(COMMAND make
  WORKING_DIRECTORY docs/latex
  )
