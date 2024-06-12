cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME map_maker)
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES CXX)

set(SOURCES src/main.cpp)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    /opt/homebrew/opt/eigen/include/eigen3
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

add_executable(${PROJECT_NAME} ${SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

