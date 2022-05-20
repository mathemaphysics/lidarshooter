This is an explanation of the file structure that the cookiecutter
generated for you:

* C++ source files:
  * `include/yolo3d/yolo3d.hpp` is the main
    C++ header that declares the interface of your library.
  * `src/yolo3d.cpp` is the main file that implements this library.
  * `app/yolo3d_app.cpp` is an executable that uses the library.
    This can e.g. be used to provide a command line interface for your project.
  * `tests/yolo3d_t.cpp` contains the unit tests for the library.
    The unit tests are written using Catch2. For further reading on what can be achieved
    with Catch2, we recommend [their tutorial](https://github.com/catchorg/Catch2/blob/devel/docs/tutorial.md).
  * `tests/tests.cpp` is the Catch2 testing driver. You do not need to change
    this. Placing this in a separate compilation unit than the unit test
    implementation decreases the compilation time of the test suite.
  * The `python/yolo3d` directory contains a Python
    package for the project. It contains a compiled Python module `_yolo3d`
    that CMake generates from the Pybind11 source file `python/yolo3d/_yolo3d.cpp`. Additionally, it contains pure Python sources (e.g. `__init__.py`) that
    allow to wrap Python functionality around the compiled module.
* CMake build system files
  * `CMakeLists.txt` describes the CMake configuration script. You can find such files
    in many directories. When CMake runs, the `CMakeLists.txt` from the top-level directory
    executes top to bottom. Whenever a command `add_subdirectory(<dir>)` is executed,
    the `CMakeLists.txt` file from the directory `<dir>` is immediately executed. A comprehensive
    reference of CMake's capabilities can be found in the [official CMake docs](https://cmake.org/documentation/).
    A well-written, opinionated book for beginners and experts is [Modern CMake](https://cliutils.gitlab.io/modern-cmake/).
* The `ext` directory contains any submodules that were added by the cookiecutter.
* Documentation configuration files
  * The Doxygen documentation is configured directly from `doc/CMakeLists.txt`.
    To further configure the build, you can check the [Doxygen Configuration Manual](https://www.doxygen.nl/manual/config.html)
    for available options and add them with `set(DOXYGEN_<param> <value>)` before
    the call to `doxygen_add_docs`.
* Configuration for CI/Code analysis/Documentation services
* Markdown files with meta information on the project. [Markdown](https://www.markdownguide.org/basic-syntax/) is
  a good language for these files, as it is easy to write and rendered into something beautiful by your git repository
  hosting provider.
  * `README.md` is the file that users will typically see first when discovering your project.
  * `COPYING.md` provides a list of copyright holders.
  * `TODO.md` contains a list of TODOs after running the cookiecutter. Following the
    instructions in that file will give you a fully functional repository with a lot
    of integration into useful web services activated and running.
  * `FILESTRUCTURE.md` describes the generated files. Feel free to remove this from the
    repository if you do not need it.
* Other files
  * `.gitignore` contains a default selection of files to omit from version control.
  * `.gitmodules` tracks the state of added submodules
  * `setup.py` describes the Python package build process. This file enables you to also
    install your software using e.g. `pip`.
  * `pyproject.toml` provides information about the Python project and its toolchain.
    `pip` uses this file before even executing `setup.py` to determine dependencies
    of the build process itself. Additionally, it can be used to configure tools like `pytest`.
  * `python/tests/test_python_bindings.py` defines a simple unit test suite for the Python
    bindings that is based on [Pytest](https://docs.pytest.org/en/stable/contents.html).
    `requirements-dev.txt` collects the required Python packages for running this
    test suite, they can be installed with `python -m pip install -r requirements-dev.txt`.
    The `pytest` run is configured in the `tools.pytest` section of `pyproject.toml`.
