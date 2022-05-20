#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "yolo3d/yolo3d.hpp"

namespace py = pybind11;

namespace yolo3d {

PYBIND11_MODULE(_yolo3d, m)
{
  m.doc() = "Python Bindings for YOLO3D";
  m.def("add_one", &add_one, "Increments an integer value");
}

} // namespace yolo3d
