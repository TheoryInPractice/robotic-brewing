# Robotics Brewing

A solver for Graph Inspection.

## Dependencies

- C++ compiler supporting the C++14 standard and [OpenMP](https://www.openmp.org/) ([GCC](https://gcc.gnu.org/) recommended)
  - If you use Mac + `clang`, run `brew install libomp` using [Homebrew](https://brew.sh/).
- [GNU Make](https://www.gnu.org/software/make/)
- [CMake](https://cmake.org/) Version 3.24 or later
- [CLI11](https://github.com/CLIUtils/CLI11) (included in this repository)
- [Gurobi Optimizer](https://www.gurobi.com/): tested with
  - Version 11.0.1 on Mac/Linux
    - You may need to recompile `libgurobi_c++.a`. See the corresponding section in this file.
  - Version 9.0.3 on Linux

### Testing dependencies

- [lcov](https://github.com/linux-test-project/lcov)
- [clang-format](https://clang.llvm.org/docs/ClangFormat.html) Version 11
  - Do not use the latest version.

## Program usage

| Operation | Command |
|:---|:---|
|Print help message | `ip-solver --help`|
|Print version | `ip-solver --version`|

## Build usage

Use `make` in this project directory.

| Operation | Command |
|:---|:---|
|Build release version | `make build` (or `make`) $\to$ executable `ip-solver` will be generated in the `build/Release/ip-solver` directory|
|Build debug version | `make build-debug` $\to$ creates `build/Debug/ip-solver`|
|Run unit tests (C++) | `make test` or `make test-cpp` |
|Run specific unit tests (C++) | `GTEST_FILTER='{TEST_NAME}.*' make test` (Replace `{TEST_NAME}` with an actual name.)|
|Run unit tests with coverage (C++) | `make test-cpp-cov` |
|Run unit tests with coverage (Python) | `make test-py` |
|Run unit tests with PyTest options (Python) | `PYTEST_OPTS='-k {TEST_NAME}' make test-py` (specific tests)<br> `PYTEST_OPTS='--full-trace' make test-py` (full trace)|
|Run unit tests with coverage (C++ and Python) | `make coverage` |
|Run unit tests with coverage (C++ and Python) and open results in HTML | `make coverage-html` |
|Clean build and coverage files | `make clean` |
|Open Jupyter Lab | `make lab` |

### How to recompile Gurobi C++ library

If you get a linker error with Gurobi, try the following steps.

- Assume Gurobi is installed in `/Library/gurobi1101/macos_universal2`.
- Using a command-line terminal, run: `cd /Library/gurobi1101/macos_universal2/src/build`.
- Edit `Makefile` accordingly.
  - If you are using `clang`, the default setting should be fine: `C++ = g++`.
  - If you are using GCC 13, set to `C++ = g++-13`.
- Run `make`.
- Copy the created file to the `lib` directory; run: `sudo cp -i libgurobi_c++.a ../../lib/`.

## Reproducing paper experiemnts

- See https://github.com/TheoryInPractice/robotic-brewing/wiki/Paper-Experiments

