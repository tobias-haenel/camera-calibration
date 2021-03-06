﻿cmake_minimum_required(VERSION 2.8)

project(camera-calibration)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

set(OpenIGTLink_DIR /usr/local/lib/igtl)

find_package(OpenCV REQUIRED)
find_package(OpenIGTLink REQUIRED)
find_package(Threads REQUIRED)

file(GLOB source_files src/*.cpp)
file(GLOB header_files include/*.h)

add_executable(${PROJECT_NAME} ${source_files} ${header_files})
add_compile_options(-Wall -pedantic -ansi -std=c++11 -pthread)

include_directories(include . ${OpenCV_INCLUDE_DIRS} ${OpenIGTLink_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${OpenIGTLink_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
