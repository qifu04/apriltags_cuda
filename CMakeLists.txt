include(ExternalProject)
include(ProcessorCount)

cmake_minimum_required(VERSION 3.16 FATAL_ERROR)  # Ensure user has a compatible version of CMake

project(ApriltagCuda LANGUAGES CUDA CXX)  # Set project name and specify the languages used

# Look for required packages.
find_package(CUDA REQUIRED)

set(GLOG_INSTALL_DIR ${CMAKE_BINARY_DIR}/glog-install)
set(GTEST_INSTALL_DIR ${CMAKE_BINARY_DIR}/gtest-install)
set(OPENCV_INSTALL_DIR ${CMAKE_BINARY_DIR}/OpenCV-install)
set(WPILIB_INSTALL_DIR ${CMAKE_BINARY_DIR}/wpilib-install)
set(CCCL_INSTALL_DIR ${CMAKE_BINARY_DIR}/cccl-install)

# Determine number of processors for the parallel builds.
if(NOT DEFINED NUM_PROCESSORS)
    ProcessorCount(NUM_PROCESSORS)
    if(NUM_PROCESSORS EQUAL 0)
        set(NUM_PROCESSORS 1)
    endif()
endif()

# Add glog package
ExternalProject_Add(
    glog
    PREFIX ${CMAKE_BINARY_DIR}/glog
    GIT_REPOSITORY https://github.com/google/glog.git
    GIT_TAG v0.7.1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${GLOG_INSTALL_DIR}
    BUILD_COMMAND make -j${NUM_PROCESSORS}
    INSTALL_COMMAND make install
)

# Add gtest package
ExternalProject_Add(
    gtest
    PREFIX ${CMAKE_BINARY_DIR}/gtest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${GTEST_INSTALL_DIR}
    BUILD_COMMAND make -j${NUM_PROCESSORS}
    INSTALL_COMMAND make install
)

# Add OPENCV package
ExternalProject_Add(
    OpenCV
    PREFIX ${CMAKE_BINARY_DIR}/OpenCV
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.9.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OPENCV_INSTALL_DIR}
    BUILD_COMMAND make -j${NUM_PROCESSORS}
    INSTALL_COMMAND make install
)

# Add wpilib package
ExternalProject_Add(
    wpilib
    PREFIX ${CMAKE_BINARY_DIR}/wpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG v2024.3.2
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${WPILIB_INSTALL_DIR} -DWITH_JAVA=OFF -DWITH_GUI=OFF -DOPENCV_INSTALL_DIR=${OPENCV_INSTALL_DIR}
    BUILD_COMMAND make -j${NUM_PROCESSORS}
    INSTALL_COMMAND make install
)

# Add CCCL package
ExternalProject_Add(
    wpilib
    PREFIX ${CMAKE_BINARY_DIR}/cccl
    GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
    GIT_TAG v2.3.2
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CCCL_INSTALL_DIR} -DCCCL_ENABLE_BENCHMARKS=OFF
    BUILD_COMMAND make
    INSTALL_COMMAND make install
)

# Add 971 apriltag package
ExternalProject_Add(
    apriltagfrc971
    PREFIX ${CMAKE_BINARY_DIR}/apriltagfrc971
    GIT_REPOSITORY https://github.com/cgpadwick/apriltag.git
    GIT_TAG v1.0
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${APRILTAG_INSTALL_DIR} -DOPENCV_INSTALL_DIR=${OPENCV_INSTALL_DIR}
    BUILD_COMMAND make -j${NUM_PROCESSORS}
    INSTALL_COMMAND make install
)

add_dependencies(wpilib OpenCV)
add_dependencies(apriltagfrc971 glog gtest OpenCV wpilib CCCL)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CUDA_COMPILER)
    message(STATUS "CUDA Compiler: ${CMAKE_CUDA_COMPILER}")

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

if(ENABLE_ASAN)
    message(STATUS "AddressSanitizer is enabled")
    set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${COMMON_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ASAN_FLAGS}")
endif()

if(ENABLE_MSAN)
    message(STATUS "MemorySanitizer is enabled")
    set(MSAN_FLAGS "-fsanitize=memory -fsanitize-memory-track-origins=2 -fno-omit-frame-pointer -fsanitize-memory-use-after-dtor")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} ${MSAN_FLAGS}")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${COMMON_FLAGS} ${MSAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MSAN_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${MSAN_FLAGS}")
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

add_dependencies(apriltag_cuda apriltagfrc971)

# Include directories for the compiler
include_directories( 
    ${apriltagfrc971_SOURCE_DIR}
    ${apriltagfrc971_SOURCE_DIR}/common/
    ${CCCL_SOURCE_DIR}/cub
    ${CCCL_SOURCE_DIR}/libcudacxx/include
    ${CCCL_SOURCE_DIR}/thrust
    ${glog_INCLUDE_DIRS}
    ${GTest_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

# Add executable for OpenCV CUDA demo
add_executable(opencv_cuda_demo opencv_cuda_demo.cu)
target_link_libraries(opencv_cuda_demo 
    apriltag_cuda 
    apriltagfrc971::apriltagfrc971
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

# Add executable for visualize
add_executable(visualize visualize.cu)
target_link_libraries(visualize
    apriltag_cuda 
    apriltagfrc971::apriltagfrc971
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

# Add the test executable
add_executable(gpu_detector_test gpu_detector_test.cu)
target_link_libraries(gpu_detector_test
    apriltag_cuda
    apriltagfrc971::apriltagfrc971
    ${OpenCV_LIBRARIES}
    glog::glog
    GTest::GTest)

# Add a custom target to format all source files
add_custom_target(format_all
    COMMAND clang-format -i -style=Google ${CMAKE_CURRENT_SOURCE_DIR}/*.cu ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    COMMENT "Running clang-format on all source files"
)
