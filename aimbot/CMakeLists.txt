cmake_minimum_required(VERSION 3.5)
project(aimbot)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SRC_FILES src/*.cpp)

add_executable(aimbot ${SRC_FILES})
target_link_libraries(aimbot ${OpenCV_LIBS})
