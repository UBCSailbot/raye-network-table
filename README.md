# Network Table
A communication hub that runs on central controller on Ada 2.0.  
Receives updates on sensor data (from GPS, wind sensors, etc), allows other programs to connect
to the network table using pub/sub or request/reply.

[Network Table Communication Protocol](https://github.com/UBCSailbot/network-table/wiki/Network-Table-Communication)

This repository also contains source code for programs which are used to transfer
data between various other programs. These are located in the `projects` folder.

## Install Dependencies
Clone all submodules:  
```git submodule update --init --recursive```  

Install these dependencies:  
```sudo apt-get install build-essential cppcheck pep8 cmake clang libzmq3-dev libboost-all-dev pkg-config libprotobuf-dev protobuf-compiler```

Python scripts require the following:  
```pip3 install pyzmq protobuf python-can```  
```pip2 install pyserial requests```

## Compiling
In the root directory of the repository run these commands:
```bash
mkdir build
cd build
cmake ..
make
```

## CMake flags (pre-compile)
When you run cmake, theres certain flags you can pass to cmake
to control how your project is compiled. Note that
these variables are "sticky". If you set them, and then
later you just run ```cmake ..```, the variables will
still be set to whatever you last set them to. You can either
pass in the flag again to set them to something else, or delete
your build folder to reset to normal.

### ROS (Robot Operating System)
```cmake .. -DENABLE_ROS:BOOL=ON```  

By default, this is set to OFF.
Setting ENABLE_ROS to ON 
builds an extra executable which is ran on the
Intel NUC. Inside the NUC, ROS is used to communicate between
different processes. Obviously you will need to have
ROS installed. Refer to the [ROS website](https://www.ros.org/install/)
to find out how to install it on your computer.

To use the sailbot-msg datatypes from your terminal, (for example if you're running
```rostopic echo /sensors```), you will have to run  
```source ./build/devel/setup.bash```

## Running
Executables can be found in `build/bin/`

## Style Guide, Linting, and Code Check
For C++ follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html).  
For Python follow [pep8](https://www.python.org/dev/peps/pep-0008/).

Run cpplint to check the code for style guideline violations.
```bash
./scripts/static_analysis/run_cpplint.sh
```

Run cppcheck to check code for possible bugs.
```bash
./scripts/static_analysis/run_cppcheck.sh
```

Run the python linter to check code for style guideline violations.
```bash
./scripts/static_analysis/run_pythoncheck.sh
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
-   **scripts/** - Scripts used for various purposes. View the comments at the top of the scripts to learn what they do.
-   **test/** - Unit tests. These test specific functionality of the network-table.
-   **projects/** - Target specific code. For more details, check the README file in this directory.
-   **jenkins/** - Jenkins scripts for running automated tests.
