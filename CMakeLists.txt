include(cmake/CPM.cmake)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)  # Ensure user has a compatible version of CMake

project(ApriltagCuda LANGUAGES CUDA CXX)  # Set project name and specify the languages used

# Look for CUDA
find_package(CUDA REQUIRED)

# Look for glog and gtest
find_package(glog REQUIRED)
find_package(GTest REQUIRED)

# Look for opencv
find_package(OpenCV 4.9.0 REQUIRED CONFIG)

 CPMAddPackage(
     NAME CCCL
     GITHUB_REPOSITORY nvidia/cccl
     GIT_TAG v2.3.2
     OPTIONS "CCCL_ENABLE_BENCHMARKS OFF"
 )

set(TEAM971_CODE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../971-Robot-Code/third_party/apriltag/)

# Gather all source files in the current directory
#file(GLOB CUDA_SOURCES "*.cu")
set(CUDA_LIB_SOURCES 
    apriltag_detect.cu 
    apriltag_gpu.cu
    cuda_frc971.cu
    labeling_allegretti_2019_BKE.cu
    line_fit_filter.cu
    points.cu
    threshold.cu)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 --expt-relaxed-constexpr -g")
#set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std c++20 -g")

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
target_link_libraries(test_cuda
    apriltag_cuda 
    apriltag 
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

add_custom_target(format_all
    COMMAND clang-format -i -style=Google ${CMAKE_CURRENT_SOURCE_DIR}/*.cu ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    COMMENT "Running clang-format on all source files"
)
