# apriltags_cuda
A standalone version of the First Robotics Team 971 cuda april tag detection code.

### Why A Standalone Version?

A few reasons:
  * don't want to pollute the 971 code base with cmake
  * the 971 code depends on a lot of internal code like AOS (Autonomous Operating System).  Clients may not want to integrate with AOS and the standalone version is simpler.
  * this is probably temporary and a better way will emerge soon.

## Prerequisites

1. Install cmake, e.g. `sudo apt install cmake` for debian based systems.

2. Install opencv with the dev libraries, e.g. `sudo apt install libopencv-dev` for debian based systems.

3. Install the google logging library glog, e.g. `sudo apt install libgoogle-glog-dev` for debian based systems.

4. Install the cuda toolkit and appropriate nvidia driver on your system.  Recommend version 11.8 or later of the cuda toolkit.  Complete instructions for cuda toolkit install for Ubuntu are here: <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#prepare-ubuntu>

5. Install clang from the LLVM project.  Recommend the automatic installation script with instructions here: <https://apt.llvm.org/> e.g. `sudo ./llvm.sh 17 all` for version 17 of clang.  If you don't want to use clang you can use nvcc (shown below).

6. Determine your cuda compute capability of your GPU as follows: `nvidia-smi --query-gpu compute_cap --format=csv` .  On embedded platforms like the Orin, `nvidia-smi` does not exist.  You need to run deviceQuery to find the compute capability.  You can build deviceQuery as follows:

```
# On the orin command line
cd /usr/local/cuda/samples/1_Utilities/deviceQuery $ sudo make $ ./deviceQuery ./deviceQuery
```

7. Build the code as follows.  Use the compute capability determined above, e.g. 7.5 translates to 75 for CMake. For clang compilation use:
   ```
    cmake -B build -DCMAKE_CUDA_COMPILER=clang++-17 -DCMAKE_CUDA_ARCHITECTURES=75
    cd build && make 
   ```
    For nvcc compilation use:
   ```
    cmake -B build -DCMAKE_CUDA_COMPILER=nvcc -DCMAKE_CUDA_ARCHITECTURES=75
    cd build && make 
   ```

   Leaving the CMAKE_BUILD_TYPE undefined will results in a Release build.  If you want a debug build add `-DCMAKE_BUILD_TYPE=Debug` to the command lines above.

If the build completes successfully you can try to run the code as shown in the next section.  If not, then try debugging what is failing by adding the VERBOSE flag to make as follows `make VERBOSE=1`.

## Running the code

Plug in a USB web cam into your system.  Then run the code as follows:

```
cd build
./opencv_cuda_demo
```

A window should pop up with the webcam feed displayed.  If you hold an april tag in front of the webcam then it should be detected and the outlines of the tag should be drawn.

## Running visualize

There is a utility called `visualize` that visualizes the imagery at a few points in the gpu detection pipeline.  To run it:

```
cd build
./visualize
```

Press any key to cycle through the different visualizations.

## Running The Tests

There is a test called gpu_detector_test.  This runs a suite of gtest test fixtures that test various parts of the code.  To run the test:

```
cd build
./gpu_detector_test
```

The output should look something like this:

```
[==========] Running 4 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 4 tests from GpuDetectorTest
[ RUN      ] GpuDetectorTest.GpuDetectsAprilTag
[       OK ] GpuDetectorTest.GpuDetectsAprilTag (179 ms)
[ RUN      ] GpuDetectorTest.GpuNoAprilTagDetections
[       OK ] GpuDetectorTest.GpuNoAprilTagDetections (114 ms)
[ RUN      ] GpuDetectorTest.CpuDetectsAprilTag
[       OK ] GpuDetectorTest.CpuDetectsAprilTag (110 ms)
[ RUN      ] GpuDetectorTest.CpuNoAprilTagDetections
[       OK ] GpuDetectorTest.CpuNoAprilTagDetections (92 ms)
[----------] 4 tests from GpuDetectorTest (496 ms total)

[----------] Global test environment tear-down
[==========] 4 tests from 1 test suite ran. (496 ms total)
[  PASSED  ] 4 tests.
```



