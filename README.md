# Network Table
A communication hub that runs on central controller on Ada 2.0.  
Receives updates on sensor data (from GPS, wind sensors, etc), allows other modules to connect
to the network table using pub/sub or request/reply.

[Network Table Communication Protocol](https://confluence.ubcsailbot.org/display/ADA2/Network+Table+Communication+Protocol)

## Install Dependencies
Install the dependencies.

#### Debian Linux (Ubuntu)
```bash
./install_deps/install_deps_linux.sh
```

#### macOS
```bash
./install_deps/install_deps_osx.sh`
```

## Building
In the root directory of the repository:
```bash
mkdir build
cd build
cmake ..
make
```

## Running
Executables can be found in `build/bin/`

## Style Guide, Linting, and Code Check
We follow [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

Run cpplint to check the code for style guideline violations.
```bash
./scripts/run_cpplint.sh
```

Run cppcheck to check code for possible bugs.
```bash
./scripts/run_cppcheck.sh
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

-   **src/** - Source code.
-   **test/** - Unit tests.
-   **projects/** - Target specific code. For more details, check out the README file in this directory.


## If you have problems
 - Check that you've initiated the git submodules.
 - Check that all dependencies are met.
 - Re-run `cmake ..`
 - If all else fails, delete your build directory and try again.

## Contributing
#### How to contribute
If your changes are significant then make a new branch that is named after the JIRA issue, in the format SOFT-XXX-brief-description

e.g. SOFT-753-boxes2headings

##### Making Changes
* Create a feature branch from where you want to base your work.
  * This is usually the master branch.
  * To quickly create a feature branch based on master; `git checkout -b
    feature/my_contribution master`.
  * **Do not work directly on the `master` branch.**
* Make commits of logical units.
* Check for unnecessary whitespace with `git diff --check` before committing.
* Make sure your commit messages are accurate and coherent.
