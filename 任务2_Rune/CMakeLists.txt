cmake_minimum_required(VERSION 3.16)

project(rune)

find_package(OpenCV REQUIRED)


add_executable(rune Rune.cpp)

target_include_directories(rune PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(rune ${OpenCV_LIBS})