cmake_minimum_required(VERSION 3.17)

project(EllipsesDetection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_definitions(_USE_MATH_DEFINES)

include("cmake/eigen.cmake")
include("cmake/opencv.cmake")

add_subdirectory(src)