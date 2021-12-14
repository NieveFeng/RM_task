cmake_minimum_required(VERSION 3.16)

project(armor_Plus)

find_package(OpenCV REQUIRED)

add_executable(armor_Plus Armor_Plus.cpp)

target_include_directories(armor_Plus PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(armor_Plus ${OpenCV_LIBS})