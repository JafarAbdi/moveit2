#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_time_optimal_trajectory_generation
{
void init_time_optimal_trajectory_generation(py::module& m);
}  // namespace bind_time_optimal_trajectory_generation
}  // namespace moveit_py
