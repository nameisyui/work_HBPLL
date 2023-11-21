cmake_minimum_required(VERSION 3.10)


add_compile_options(-std=c++17 -Ofast -lpthread)
include_directories(E:/C/sjjg)
include_directories(D:/boost_1_83_0/boost_1_83_0)
add_executable(run main.cpp)
set_target_properties(run PROPERTIES RUNTIME_OUTPUT_DIRECTORY E:/C/sjjg)