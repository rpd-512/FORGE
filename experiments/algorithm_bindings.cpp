#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "../metaheuristics/teachingLearningBasedOptimization.h"
#include "../metaheuristics/socialGroupOptimization.h"
#include "../metaheuristics/geneticAlgorithm.h"
#include "../metaheuristics/particleSwarmOptimization.h"
#include "../metaheuristics/differentialEvolution.h"
#include "../gradientDescent/gradientDescent.h"
#include "../src/random_utils.h"
#include "../src/robomath_utils.h"
#include "../src/types.h"
#include "../src/nearest_neighbor_utils.h"

namespace py = pybind11;

PYBIND11_MODULE(algorithm_modules_cpp, m) {
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
    py::class_<BoxObject>(m, "BoxObject")
        .def(py::init<>())
        .def_readwrite("min_corner", &BoxObject::min_corner)
        .def_readwrite("max_corner", &BoxObject::max_corner);
    py::class_<SphereObject>(m, "SphereObject")
        .def(py::init<>())
        .def_readwrite("center", &SphereObject::center)
        .def_readwrite("radius", &SphereObject::radius);
    py::class_<CylinderObject>(m, "CylinderObject")
        .def(py::init<>())
        .def_readwrite("base_center", &CylinderObject::base_center)
        .def_readwrite("radius", &CylinderObject::radius)
        .def_readwrite("height", &CylinderObject::height);

    py::class_<position3D>(m, "position3D")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &position3D::x)
        .def_readwrite("y", &position3D::y)
        .def_readwrite("z", &position3D::z);

    py::class_<dh_param>(m, "dh_param")
        .def(py::init<float, float, float>(), py::arg("a"), py::arg("d"), py::arg("alpha"))
        .def_readwrite("a", &dh_param::a)
        .def_readwrite("d", &dh_param::d)
        .def_readwrite("alpha", &dh_param::alpha);

    py::class_<SceneObjectData>(m, "SceneObjectData")
        .def(py::init<>())
        .def_readwrite("box", &SceneObjectData::box)
        .def_readwrite("sphere", &SceneObjectData::sphere)
        .def_readwrite("cylinder", &SceneObjectData::cylinder);
    
    py::class_<SceneObject>(m, "SceneObject")
        .def(py::init<>())
        .def_readwrite("type", &SceneObject::type)
        .def_readwrite("data", &SceneObject::data);
    
    py::class_<NearestNeighbourIndex>(m, "NearestNeighbourIndex")
        .def(py::init<RobotInfo>(), py::arg("robot"))
        .def("insert", &NearestNeighbourIndex::insert, "Insert a new data point into the index")
        .def("query", &NearestNeighbourIndex::query, "Query the nearest neighbors for a given point")
        .def("rebuild", &NearestNeighbourIndex::rebuild, "Rebuild the index to optimize performance")
        .def("get_balance_score", &NearestNeighbourIndex::get_balance_score, "Get the balance score of the index")
        .def("get_max_depth", &NearestNeighbourIndex::get_max_depth, "Get the maximum depth of the index")
        .def("get_min_depth", &NearestNeighbourIndex::get_min_depth, "Get the minimum depth of the index");
        

    m.def("forward_kinematics", &forward_kinematics, "Calculate forward kinematics for a robot given its joint angles and DH parameters");
    m.def("particleSwarmOptimization", &particleSwarmOptimization, "Particle Swarm Optimization Algorithm");
    m.def("geneticAlgorithm", &geneticAlgorithm, "Genetic Algorithm");
    m.def("socialGroupOptimization", &socialGroupOptimization, "Social Group Optimization Algorithm");
    m.def("teachingLearningBasedOptimization", &teachingLearningBasedOptimization, "Teaching Learning Based Optimization Algorithm");
    m.def("differentialEvolutionAlgorithm", &differentialEvolutionAlgorithm, "Differential Evolution Algorithm");
    m.def("gradientDescent", &gradientDescent, "Gradient Descent Algorithm");
    m.def("generateChromosome", &generateChromosome, "Generate random chromosomes for the population");
}