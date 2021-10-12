cmake_minimum_required(VERSION 3.16)

project(pnp)

find_package(OpenCV REQUIRED)

add_executable(pnp pnp.cpp)

target_include_directories(pnp PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(pnp ${OpenCV_LIBS})