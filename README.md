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

5. Install clang from the LLVM project.  Recommend the automatic installation script with instructions here: <https://apt.llvm.org/> e.g. `sudo ./llvm.sh 17 all` for version 17 of clang. 


6. Clone the First Robotics Team 971 repository here: git@github.com:frc971/971-Robot-Code.git
    ```
    mkdir -p ~/code
    cd ~/code
    git clone git@github.com:frc971/971-Robot-Code.git
    ```

7. Build the 971 fork of the UMich april tag repository as follows:
    ```
    cd ~/code/971-Robot-Code/third_party/apriltag
    cmake -B build
    cd build && make
    ```

8. Edit the CMakeLists.txt file in the current directory and update the TEAM971_CODE_ROOT variable to point to the right location.

    ```
    set(TEAM971_CODE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../971-Robot-Code/third_party/apriltag/)
    ```

9. Determine your cuda compute capability of your GPU as follows: `nvidia-smi --query-gpu compute_cap --format=csv` 

10. Build the code as follows (use the compute capability determined above, e.g. 7.5 translates to 75 for CMake):
   ```
    cmake -B build -DCMAKE_CUDA_COMPILER=clang++-17 -DCMAKE_CUDA_ARCHITECTURES=75
    cd build && make 
   ```

If the build completes successfully you can try to run the code as shown in the next section.  If not, then try debugging what is failing by adding the VERBOSE flag to make as follows `make VERBOSE=1`.

## Running the code

Plug in a USB web cam into your system.  Then run the code as follows:

```
cd build
./opencv_cuda_demo
```

A window should pop up with the webcam feed displayed.  If you hold an april tag in front of the webcam then it should be detected and the outlines of the tag should be drawn.



