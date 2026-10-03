cmake_minimum_required(VERSION 3.20)

project(project_3_vo LANGUAGES CXX)

find_package(OpenCV CONFIG REQUIRED)

add_executable(project_3_vo 
    src/main.cpp 
    src/feature.cpp 
    src/matching.cpp 
    src/geometry.cpp 
    src/camera_calibration.cpp
)

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

# Camera calibration tool
add_executable(camera_calibrate
    src/calibration_main.cpp
)

target_compile_features(camera_calibrate
    PRIVATE
    cxx_std_17
)

if(MSVC)
    target_compile_options(camera_calibrate
        PRIVATE
        /utf-8
    )
endif()

target_include_directories(camera_calibrate
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_calibrate
    PRIVATE
    ${OpenCV_LIBS}
)