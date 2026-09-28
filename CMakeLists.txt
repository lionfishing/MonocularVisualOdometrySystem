cmake_minimum_required(VERSION 3.20)

project(project_3_vo LANGUAGES CXX)

find_package(OpenCV CONFIG REQUIRED)

add_executable(project_3_vo src/main.cpp src/feature.cpp)

target_compile_features(project_3_vo PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(project_3_vo
        PRIVATE
        /utf-8
    )
endif()

target_include_directories(project_3_vo 
    PRIVATE 
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(project_3_vo PRIVATE ${OpenCV_LIBS})