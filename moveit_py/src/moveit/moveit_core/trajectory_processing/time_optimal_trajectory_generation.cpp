#include "time_optimal_trajectory_generation.h"
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>

namespace moveit_py
{
namespace bind_time_optimal_trajectory_generation
{
void init_time_optimal_trajectory_generation(py::module& m)
{
  py::module totg = m.def_submodule("time_optimal_trajectory_generation");

  py::class_<trajectory_processing::TimeOptimalTrajectoryGeneration,
             std::shared_ptr<trajectory_processing::TimeOptimalTrajectoryGeneration>>(totg,
                                                                                      "TimeOptimalTrajectoryGeneration")
      .def(py::init<double, double, double>(), py::arg("path_tolerance") = 0.1, py::arg("resample_dt") = 0.1,
           py::arg("min_angle_change") = 0.001)
      .def("compute_time_stamps",
           py::overload_cast<robot_trajectory::RobotTrajectory&, const double, const double>(
               &trajectory_processing::TimeOptimalTrajectoryGeneration::computeTimeStamps, py::const_),
           py::arg("trajectory"), py::arg("max_velocity_scaling_factor") = 1.0,
           py::arg("max_acceleration_scaling_factor") = 1.0);
}
}  // namespace bind_time_optimal_trajectory_generation
}  // namespace moveit_py
