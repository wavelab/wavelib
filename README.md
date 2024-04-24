# libwave
[![Build Status](https://travis-ci.org/wavelab/libwave.png?branch=master)][1]

This library contains reusable code for:

- Estimation
- Kinematics
- Optimization
- Vision
- and more ..

## Requirements

`libwave` is divided into modules which can be built separately.
Each has its own dependecies.

| Module             | Dependencies |
| ------------------ | ------------ |
| wave\_benchmark    | wave\_containers, wave\_geometry, wave\_utils |
| wave\_containers   | Eigen, Boost |
| wave\_controls     | Eigen |
| wave\_geography    | Eigen, GeographicLib |
| wave\_geometry     | Eigen, Boost |
| wave\_gtsam        | wave\_utils, gtsam |
| wave\_kinematics   | wave\_utils, wave\_controls |
| wave\_matching     | wave\_utils, Boost, PCL  |
| wave\_optimization | wave\_utils, wave\_kinematics, wave\_vision, Ceres |
| wave\_utils        | Eigen, yaml-cpp |
| wave\_vision       | wave\_utils, wave\_kinematics, wave\_containers, Boost Filesystem, OpenCV (calib3d, features2d, highgui, imgproc, videoio) |


The following versions are the minimum we test against.
Some earlier versions may work, but are not tested.

- Boost 1.54
- Ceres 1.13
- Eigen 3.3.2
- OpenCV 4.2.0
- PCL 1.8
- yaml-cpp 0.5.1
- gtsam
- GeographicLib 1.49

Building libwave requires CMake 3.16 and a C++17 compiler (tested on GCC 5.4).

### Installing dependencies
The basic set of dependencies can be installed with the Ubuntu package manager
using the command

```
# apt-get install libboost-dev libyaml-cpp-dev libeigen3-dev build-essential cmake
```

For convenience, scripts to install other dependencies on Ubuntu 16.04 are
provided in `scripts/install`. **Note**: the scripts are not tested on a wide
variety of systems. They may be incompatible with other packages installed on
your system.

## Install from source

Clone the repo with submodules:

    git clone --recursive https://github.com/wavelab/libwave.git

Install the dependencies required for the modules you want to build, as
described above. Then build using CMake:

    cd libwave
    mkdir -p build
    cd build
    cmake ..
    make -j8

By default, all libraries whose dependencies are found will be built. Individual
libraries can be disabled using CMake options. For example,

    cmake .. -DBUILD_wave_vision=OFF

will disable building `wave_vision`.

Install libwave by running `make install`. Alternatively, you can enable the
`EXPORT_BUILD` option in CMake, which will make the libwave build directory
searchable by CMake without installation.


**Known issue with OpenCV**: `wave_vision` may
[fail to build](https://github.com/wavelab/libwave/issues/267) if OpenCV 2 is
installed in the default system directory on Ubuntu 16.04 (and OpenCV 3 is
installed elsewhere). A workaround is to remove OpenCV 2 via
`sudo apt remove libopencv-dev`.


## Use with CMake

One libwave has been either installed or exported by CMake, it can be used in
your project's `CMakeLists.txt` file as follows:

    cmake_minimum_required(VERSION 3.16)
    project(example)

    find_package(wave REQUIRED geometry matching)

    add_executable(example example.cpp)
    target_link_libraries(example wave::geometry wave::matching)


## Documentation

The documentation consists of an API reference as well as pages on Fundamentals,
describing concepts and theory behind the library.

It is generated using Doxygen, using the build target `make doc`.


## Notes for Developers

To expedite our development process we use a set of git hooks to run:

- Clang format (pre-commit)

these can be activated by running the `githook_init.bash` from the root of the repo

    bash scripts/githooks/githooks_init.bash


## LICENSE

Copyright (c) 2017 Waterloo Autonomous Vehicles Laboratory (WAVELab)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

[1]: https://travis-ci.org/wavelab/libwave
[edit_docs]: http://chutsu.github.io/ditto/#docs/how_do_i_use_ditto
