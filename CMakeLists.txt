cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME map_maker)
project(${PROJECT_NAME} VERSION 1.0.0 LANGUAGES CXX)

set(SOURCES src/main.cpp)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

add_executable(${PROJECT_NAME} ${SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
if (UNIX)
   target_link_libraries(${PROJECT_NAME} pthread)
endif (UNIX)
