cmake_minimum_required(VERSION 3.16)

project(armor)

find_package(OpenCV REQUIRED)

add_executable(armor Armor.cpp)

target_include_directories(armor PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(armor ${OpenCV_LIBS})