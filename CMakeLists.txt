cmake_minimum_required(VERSION 3.5)

project(ParallelCBS)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# clang-tidy target (linter & static code analysis)
add_custom_target(clang-tidy
  COMMAND CMAKE_EXPORT_COMPILE_COMMANDS=ON run-clang-tidy ${CMAKE_CURRENT_SOURCE_DIR})

# clang-format
set(ALL_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cbs.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/timer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/cbs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/example/timer.hpp
)

add_custom_target(clang-format
  COMMAND clang-format -i ${ALL_SOURCE_FILES}
)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -pthread")

# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
)

# Examples

## cbs
add_executable(cbs
  example/cbs.cpp
)
target_link_libraries(cbs
  ${Boost_LIBRARIES}
  yaml-cpp
)
