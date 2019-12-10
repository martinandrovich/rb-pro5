cmake_minimum_required(VERSION 3.10)
project (Q_navigation)

file(GLOB_RECURSE sources "src/*.cpp")
file(GLOB_RECURSE headers "src/*.hpp")

set (includes "")
foreach (_headerFile ${headers})
  get_filename_component(_dir ${_headerFile} PATH)
  list (APPEND includes ${_dir})
endforeach()
list(REMOVE_DUPLICATES includes)

find_package( OpenCV REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_executable(Q_navigation main.cpp ${sources} )
target_link_libraries(Q_navigation PRIVATE ${OpenCV_LIBS} )
