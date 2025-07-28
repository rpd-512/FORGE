#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "../src/collision_utils.h"
#include "../src/types.h"

namespace py = pybind11;

PYBIND11_MODULE(collision_cpp, m) {
    py::class_<position3D>(m, "position3D")
        .def(py::init([](float x, float y, float z) {
            return position3D{x, y, z};
        }))
        .def_readwrite("x", &position3D::x)
        .def_readwrite("y", &position3D::y)
        .def_readwrite("z", &position3D::z);

    m.def("line_intersects_aabb", &line_intersects_aabb);
    m.def("line_intersects_sphere", &line_intersects_sphere);
    m.def("line_intersects_cylinder", &line_intersects_cylinder);
}