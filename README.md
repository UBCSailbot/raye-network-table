# Network Table
A communication hub that runs on central controller on Ada 2.0.  
Receives updates on sensor data (from GPS, wind sensors, etc), allows other programs to connect
to the network table using pub/sub or request/reply.

[Network Table Communication Protocol](https://ubcsailbot.atlassian.net/wiki/spaces/ADA2/pages/1235622/Network+Table+Communication+Protocol)

This repository also contains source code for programs which are used to transfer
data between various other programs. These are located in the `projects` folder.

## Install Dependencies
Clone all submodules:  
```git submodule update --init --recursive```  

Install these dependencies (all should be available via apt-get):  
```build-essential cppcheck pep8 cmake clang libzmq3-dev libboost-all-dev pkg-config```

The following dependencies are needed to build protobuf.
They're pretty common anyways, you probably have a lot of them:  
```autoconf automake libtool curl make g++ unzip```

Run this script to compile and install protobuf
 locally in the lib folder. **warning**, 
this script takes about 1 hour to run.  
```./scripts/install_protobuf.sh```

### ROS (Robot Operating System)
You can enable ROS when running cmake by running:  
```cmake .. -DENABLE_ROS:BOOL=ON```
From then on, ROS will be enabled, even if you run cmake again.
To turn it off, run:  
```cmake .. -DENABLE_ROS:BOOL=OFF```

If ENABLE_ROS is set to ON in the top
level CMakeLists.txt, this builds an
extra executable which is ran on the
Intel NUC. Obviously you will need to have
ROS and catkin installed. Refer to the [ROS website](https://www.ros.org/install/)
to find out how to install it on your computer.

## Compiling
In the root directory of the repository run these commands:
```bash
mkdir build
cd build
cmake ..
make
```

## Running
Executables can be found in `build/bin/`

## Style Guide, Linting, and Code Check
For C++ follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html).  
For Python follow [pep8](https://www.python.org/dev/peps/pep-0008/).

Run cpplint to check the code for style guideline violations.
```bash
./scripts/run_cpplint.sh
```

Run cppcheck to check code for possible bugs.
```bash
./scripts/run_cppcheck.sh
```

Run the python linter to check code for style guideline violations.
```bash
./scripts/run_pythoncheck.sh
```

## Testing
##### Creating & Running Tests
Whenever you add new tests, you will need to add the required `.cpp` and `.h` files to the `TEST_FILES` parameter in `test/basic_tests/CMakelists.txt`.

For example:
```cmake
set(TEST_FILES
    example_tests/NewTest.cpp
    example_tests/NewTest.h)
```

To run the tests, use CMake to compile the project, then run `basic_tests` in `build/bin`.
It will notify you if tests pass or fail.

## Structure of the code
The directories `src` and `test/basic_tests` should mirror each other. That is, any unit testing code for the file `src/a/b.cpp` should be in `test/basic_tests/a/bTest.cpp`.

-   **src/** - Source code of the network-table.
-   **scripts/** - Scripts used for various purposes. These can just be directly ran and do not require compiling.
-   **test/** - Unit tests. These test specific functionality of the network-table.
-   **projects/** - Target specific code. For more details, check out the README file in this directory.
