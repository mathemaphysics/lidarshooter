# Welcome to LiDARShooter



# Prerequisites

Building LiDARShooter requires the following software installed:

* A C++17-compliant compiler
* CMake `>= 3.19`
* Doxygen (optional, documentation building is skipped if missing) Python `>= 3.6` for building Python bindings

# Building LiDARShooter

The following sequence of commands builds LiDARShooter.
It assumes that your current working directory is the top-level directory
of the freshly cloned repository:

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

The build process can be customized with the following CMake variables,
which can be set by adding `-D<var>={ON, OFF}` to the `cmake` call:

* `BUILD_TESTING`: Enable building of the test suite (default: `ON`)
* `BUILD_DOCS`: Enable building the documentation (default: `ON`)
* `BUILD_PYTHON`: Enable building the Python bindings (default: `ON`)


If you wish to build and install the project as a Python project without
having access to C++ build artifacts like libraries and executables, you
can do so using `pip` from the root directory:

```
python -m pip install .
```

# Testing LiDARShooter

When built according to the above explanation (with `-DBUILD_TESTING=ON`),
the C++ test suite of `LiDARShooter` can be run using
`ctest` from the build directory:

```
cd build
ctest
```

# Documentation

LiDARShooter provides a Doxygen documentation. You can build
the documentation locally by making sure that `Doxygen` is installed on your system
and running this command from the top-level build directory:

```
cmake --build . --target doxygen
```

The web documentation can then be browsed by opening `doc/html/index.html` in your browser.
