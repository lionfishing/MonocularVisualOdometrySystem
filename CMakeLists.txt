cmake_minimum_required(VERSION 3.20)

project(project_3_vo LANGUAGES CXX)

add_executable(project_3_vo src/main.cpp)

target_compile_features(project_3_vo PRIVATE cxx_std_17)