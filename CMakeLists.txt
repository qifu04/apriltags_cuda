include(cmake/CPM.cmake)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)  # Ensure user has a compatible version of CMake

project(ApriltagCuda LANGUAGES CUDA CXX)  # Set project name and specify the languages used

# Look for required packages.
find_package(CUDA REQUIRED)
find_package(glog REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCV 4.9.0 REQUIRED CONFIG)

 CPMAddPackage(
     NAME CCCL
     GITHUB_REPOSITORY nvidia/cccl
     GIT_TAG v2.3.2
     OPTIONS "CCCL_ENABLE_BENCHMARKS OFF"
 )

set(TEAM971_CODE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../971-Robot-Code/third_party/apriltag/)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CUDA_COMPILER)
    message(STATUS "CUDA Compiler: ${CMAKE_CUDA_COMPILER}")

    # You can add specific actions based on the CUDA compiler here
    if(CMAKE_CUDA_COMPILER MATCHES "nvcc")
        message(STATUS "Using NVCC as CUDA compiler")
        if(CMAKE_BUILD_TYPE MATCHES "Debug")
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 --expt-relaxed-constexpr -g")
        else()
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 --expt-relaxed-constexpr -Xptxas -O3")
        endif()
    else()
        message(STATUS "Using a different CUDA compiler")
        if(CMAKE_BUILD_TYPE MATCHES "Debug")
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -g")
        else()
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -O3")
        endif()
    endif()
else()
    message(FATAL_ERROR "CUDA compiler not found. Please set CMAKE_CUDA_COMPILER.")
endif()

# Gather all source files in the current directory
set(CUDA_LIB_SOURCES 
    apriltag_detect.cu 
    apriltag_gpu.cu
    cuda_frc971.cu
    labeling_allegretti_2019_BKE.cu
    line_fit_filter.cu
    points.cu
    threshold.cu
    apriltag_utils.cu)

# Add a library with the above source files
add_library(apriltag_cuda ${CUDA_LIB_SOURCES})

# Include directories for the compiler
include_directories( 
    ${TEAM971_CODE_ROOT}
    ${TEAM971_CODE_ROOT}/common/
    ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/cccl-src/cub
    ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/cccl-src/libcudacxx/include
    ${CMAKE_CURRENT_SOURCE_DIR}/build/_deps/cccl-src/thrust
    ${OpenCV_INCLUDE_DIRS})


message(${OpenCV_INCLUDE_DIRS})
link_directories(${TEAM971_CODE_ROOT}/build)

add_executable(opencv_cuda_demo
    opencv_cuda_demo.cu)
target_link_libraries(opencv_cuda_demo 
    apriltag_cuda 
    apriltag 
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

add_executable(visualize visualize.cu)
target_link_libraries(visualize
    apriltag_cuda 
    apriltag 
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

# Add the test executable
add_executable(gpu_detector_test gpu_detector_test.cu)
target_link_libraries(gpu_detector_test
    apriltag_cuda
    apriltag
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

add_custom_target(format_all
    COMMAND clang-format -i -style=Google ${CMAKE_CURRENT_SOURCE_DIR}/*.cu ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    COMMENT "Running clang-format on all source files"
)
