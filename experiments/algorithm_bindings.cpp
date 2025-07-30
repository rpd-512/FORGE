#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "../metaheuristics/teachingLearningBasedOptimization.h"
#include "../metaheuristics/socialGroupOptimization.h"
#include "../metaheuristics/geneticAlgorithm.h"
#include "../metaheuristics/particleSwarmOptimization.h"
#include "../metaheuristics/differentialEvolution.h"
#include "../gradientDescent/gradientDescent.h"

namespace py = pybind11;

PYBIND11_MODULE(designer_modules_cpp, m) {
    m.doc() = "FORGE: Formation of Optimized Robotic Groundtruth Examples";
    py::class_<plotPoint>(m, "PlotPoint")
        .def(py::init<>())
        .def_readwrite("name", &plotPoint::name)
        .def_readwrite("fitness", &plotPoint::fitness)
        .def_readwrite("best_gene", &plotPoint::best_gene)
        .def_readwrite("fitness_history", &plotPoint::fitness_history)
        .def_readwrite("distance_history", &plotPoint::distance_history)
        .def_readwrite("angular_history", &plotPoint::angular_history);
    py::class_<RobotInfo>(m, "RobotInfo")
        .def(py::init<>())
        .def_readwrite("dof", &RobotInfo::dof)
        .def_readwrite("name", &RobotInfo::name)
        .def_readwrite("dh_params", &RobotInfo::dh_params)
        .def_readwrite("joint_angle", &RobotInfo::joint_angle)
        .def_readwrite("destination", &RobotInfo::destination)
        .def_readwrite("init_pos", &RobotInfo::init_pos)
        .def_readwrite("scene_objects", &RobotInfo::scene_objects);
    
    m.def("particleSwarmOptimization", &particleSwarmOptimization, "Particle Swarm Optimization Algorithm");
    m.def("geneticAlgorithm", &geneticAlgorithm, "Genetic Algorithm");
    m.def("socialGroupOptimization", &socialGroupOptimization, "Social Group Optimization Algorithm");
    m.def("teachingLearningBasedOptimization", &teachingLearningBasedOptimization, "Teaching Learning Based Optimization Algorithm");
    m.def("differentialEvolutionAlgorithm", &differentialEvolutionAlgorithm, "Differential Evolution Algorithm");
    m.def("gradientDescent", &gradientDescent, "Gradient Descent Algorithm");
}