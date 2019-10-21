cmake_minimum_required(VERSION 3.10.2)

# project name
project(rb-pro5)

# project settings
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# warning levels
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic")

# source folder (.cpp + .h files)
file(GLOB SOURCES src/*.cpp)
file(GLOB SOURCES src/modules/*.cpp)
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/src/modules/)

# executable
add_executable(${PROJECT_NAME} src/main.cpp ${SOURCES})

# dependencies

## opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

## gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${GAZEBO_PROTO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${GAZEBO_PROTO_LIBRARIES})

## python libs
#find_package(PythonLibs 2.7)
#include_directories(${PYTHON_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})

## fuzzylite (static)
include_directories(${PROJECT_SOURCE_DIR}/lib/fuzzylite/)
target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/lib/fuzzylite/bin/libfuzzylite-static.a)
