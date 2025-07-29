#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h> 
#include "../src/robomath_utils.h"
#include "../src/collision_utils.h"
#include "../src/types.h"

namespace py = pybind11;

PYBIND11_MODULE(designer_modules_cpp, m) {
    py::class_<position3D>(m, "position3D")
        .def(py::init([](float x, float y, float z) {
            return position3D{x, y, z};
        }))
        .def_readwrite("x", &position3D::x)
        .def_readwrite("y", &position3D::y)
        .def_readwrite("z", &position3D::z);

    py::class_<dh_param>(m, "dh_param")
        .def(py::init([](double a, double d, double alpha) {
            return dh_param{a, d, alpha};
        }), py::arg("a"), py::arg("d"), py::arg("alpha"))
        .def_readwrite("a", &dh_param::a)
        .def_readwrite("d", &dh_param::d)
        .def_readwrite("alpha", &dh_param::alpha);

    py::class_<RobotInfo>(m, "RobotInfo")
        .def(py::init([](int dof, const std::vector<dh_param>& dh_params) {
            RobotInfo r;
            r.dof = dof;
            r.dh_params = dh_params;
            return r;
        }), py::arg("dof"), py::arg("dh_params"))
        .def_readwrite("dof", &RobotInfo::dof)
        .def_readwrite("dh_params", &RobotInfo::dh_params);
        
        m.def("line_intersects_aabb", &line_intersects_aabb);
        m.def("line_intersects_sphere", &line_intersects_sphere);
        m.def("line_intersects_cylinder", &line_intersects_cylinder);
        m.def("forward_kinematics", &forward_kinematics);
        m.def("dh_transform", &dh_transform_py);
}