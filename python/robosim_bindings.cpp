#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "robosim/env/environment.h"
#include "robosim/parallel/env_pool.h"
#include "robosim/robot/robot_factory.h"
#include "robosim/checkpoint/state_snapshot.h"
#include "robosim/ai/onnx_inference.h"

namespace py = pybind11;
using namespace robosim;

PYBIND11_MODULE(_robosim_core, m) {
    m.doc() = "RoboSim: Research-grade robot simulation engine";

    // Ensure robots are registered
    robot::RobotFactory::register_all_robots();

    // --- Robot Info ---
    py::class_<robot::RobotInfo>(m, "RobotInfo")
        .def_readonly("id", &robot::RobotInfo::id)
        .def_readonly("display_name", &robot::RobotInfo::display_name)
        .def_readonly("description", &robot::RobotInfo::description)
        .def_readonly("num_joints", &robot::RobotInfo::num_joints)
        .def_readonly("weight_kg", &robot::RobotInfo::weight_kg);

    // --- Robot Factory ---
    m.def("available_robots", []() {
        return robot::RobotFactory::instance().available_robots();
    });

    // --- Single Environment ---
    py::class_<env::Environment>(m, "Environment")
        .def(py::init<const std::string&, const std::string&, bool>(),
             py::arg("robot_name"), py::arg("task_name") = "walk_forward",
             py::arg("headless") = true)
        .def("step", [](env::Environment& self, const Eigen::VectorXd& action) {
            auto result = self.step(action);
            return py::make_tuple(result.observation, result.reward,
                                  result.terminated, result.truncated, result.info);
        }, py::call_guard<py::gil_scoped_release>())
        .def("reset", [](env::Environment& self, std::optional<unsigned int> seed) {
            auto result = self.reset(seed);
            return py::make_tuple(result.observation, result.info);
        }, py::arg("seed") = py::none(),
           py::call_guard<py::gil_scoped_release>())
        .def("close", &env::Environment::close)
        .def_property_readonly("observation_dim", &env::Environment::observation_dim)
        .def_property_readonly("action_dim", &env::Environment::action_dim)
        .def_property_readonly("step_count", &env::Environment::step_count);

    // --- Vectorized Environment Pool ---
    py::class_<parallel::EnvPool>(m, "EnvPool")
        .def(py::init<const std::string&, const std::string&, int, int>(),
             py::arg("robot_name"), py::arg("task_name"),
             py::arg("num_envs"), py::arg("num_threads") = 0)
        .def("reset", &parallel::EnvPool::reset,
             py::call_guard<py::gil_scoped_release>())
        .def("step", &parallel::EnvPool::step,
             py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("num_envs", &parallel::EnvPool::num_envs)
        .def_property_readonly("observation_dim", &parallel::EnvPool::observation_dim)
        .def_property_readonly("action_dim", &parallel::EnvPool::action_dim);

    // --- Batched State ---
    py::class_<parallel::BatchedState>(m, "BatchedState")
        .def_readonly("observations", &parallel::BatchedState::observations)
        .def_readonly("rewards", &parallel::BatchedState::rewards)
        .def_readonly("dones", &parallel::BatchedState::dones)
        .def_readonly("truncateds", &parallel::BatchedState::truncateds);

    // --- ONNX Inference ---
    py::class_<ai::OnnxInference>(m, "OnnxInference")
        .def(py::init<>())
        .def("load_model", &ai::OnnxInference::load_model)
        .def("infer", &ai::OnnxInference::infer,
             py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("is_loaded", &ai::OnnxInference::is_loaded)
        .def_property_readonly("input_dim", &ai::OnnxInference::input_dim)
        .def_property_readonly("output_dim", &ai::OnnxInference::output_dim);

    // --- Checkpoint ---
    py::class_<checkpoint::StateSnapshot>(m, "StateSnapshot")
        .def(py::init<>())
        .def("save", &checkpoint::StateSnapshot::save)
        .def("load", &checkpoint::StateSnapshot::load)
        .def_readwrite("simulation_time", &checkpoint::StateSnapshot::simulation_time)
        .def_readwrite("step_count", &checkpoint::StateSnapshot::step_count);
}
