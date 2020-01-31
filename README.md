# av1dec
experimental av1 decoder

# License
av1dec is available under the terms of The 2-Clause BSD License. Files under aom directory copied from libaom project. They are license under 2-Clause BSD and Alliance for Open Media Patent License 1.0 

# Known Limitations
  * Supports only 8-bits
  * Only 4:2:0 content are tested.
  * No Intrabc
  * No SVC
  * No segmentation
  * The conformance test scipt only supports windows.
  
# Build
## Windows* Operating Systems (64-bit)
- __Build Requirements__
  - Visual Studio (download [here](https://visualstudio.microsoft.com/downloads/))
  - CMake 3.5 or later (download [here](https://github.com/Kitware/CMake/releases/download/v3.14.5/cmake-3.14.5-win64-x64.msi))
- __Build Steps__
  - mkdir build && cd build && cmake ..
  - open av1dec.sln under build directory, build av1dec project using the visual studio IDE
  - the the av1dec.exe under build/tests

## Linux* Operating Systems (64-bit)
- __Build Requirements__
  - GCC 5.4.0 or later
  - CMake 3.5.1 or later
- __Build Steps__
  - mkdir build && cd build && cmake .. && make -j32
  - run the av1dec under build

# Code style
- We general follows WebKit coding style in C++ code: http://www.webkit.org/coding/coding-style.html
- You can use git-clang-format --style=WebKit HEAD^ to format you patch

# Testing
please make run testscipt/conformance.py, and make all cases passed before you send your patch.
