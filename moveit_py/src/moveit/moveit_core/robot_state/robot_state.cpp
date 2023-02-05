/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter David Fagan */

#include "robot_state.h"
#include <pybind11/stl.h>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/docstring/moveit_robot_state.hpp>
#include <moveit/robot_state/conversions.h>

namespace moveit_py
{
namespace bind_robot_state
{
void update(std::shared_ptr<moveit::core::RobotState>& robot_state, bool force, std::string& category)
{
  if (category == "all")
  {
    robot_state->update(force);
  }
  else if (category == "links_only")
  {
    robot_state->updateLinkTransforms();
  }
  else if (category == "collisions_only")
  {
    robot_state->updateCollisionBodyTransforms();
  }
  else
  {
    throw std::invalid_argument("Invalid category");
  }
}

Eigen::MatrixXd get_frame_transform(std::shared_ptr<moveit::core::RobotState>& robot_state, std::string& frame_id)
{
  bool frame_found;
  auto transformation = robot_state->getFrameTransform(frame_id, &frame_found);
  return transformation.matrix();
}

Eigen::MatrixXd get_global_link_transform(std::shared_ptr<moveit::core::RobotState>& robot_state, std::string& link_name)
{
  auto transformation = robot_state->getGlobalLinkTransform(link_name);
  return transformation.matrix();
}

geometry_msgs::msg::Pose get_pose(std::shared_ptr<moveit::core::RobotState>& robot_state, const std::string& link_name)
{
  Eigen::Isometry3d pose = robot_state->getGlobalLinkTransform(link_name);
  return tf2::toMsg(pose);
}

std::map<std::string, double> get_joint_positions(std::shared_ptr<moveit::core::RobotState>& robot_state)
{
  std::map<std::string, double> joint_positions;
  const std::vector<std::string>& variable_name = robot_state->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_positions[name.c_str()] = robot_state->getVariablePosition(name);
  }
  return joint_positions;
}

void set_joint_positions(std::shared_ptr<moveit::core::RobotState>& robot_state,
                         std::map<std::string, double>& joint_positions)
{
  for (const auto& item : joint_positions)
  {
    robot_state->setVariablePosition(item.first, item.second);
  }
}

std::map<std::string, double> get_joint_velocities(std::shared_ptr<moveit::core::RobotState>& robot_state)
{
  std::map<std::string, double> joint_velocity;
  const std::vector<std::string>& variable_name = robot_state->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_velocity[name.c_str()] = robot_state->getVariableVelocity(name);
  }
  return joint_velocity;
}

void set_joint_velocities(std::shared_ptr<moveit::core::RobotState>& robot_state,
                          std::map<std::string, double>& joint_velocities)
{
  for (const auto& item : joint_velocities)
  {
    robot_state->setVariableVelocity(item.first, item.second);
  }
}

std::map<std::string, double> get_joint_accelerations(std::shared_ptr<moveit::core::RobotState>& robot_state)
{
  std::map<std::string, double> joint_acceleration;
  const std::vector<std::string>& variable_name = robot_state->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_acceleration[name.c_str()] = robot_state->getVariableAcceleration(name);
  }
  return joint_acceleration;
}

void set_joint_accelerations(std::shared_ptr<moveit::core::RobotState>& robot_state,
                             std::map<std::string, double>& joint_accelerations)
{
  for (const auto& item : joint_accelerations)
  {
    robot_state->setVariableAcceleration(item.first, item.second);
  }
}

std::map<std::string, double> get_joint_efforts(std::shared_ptr<moveit::core::RobotState>& robot_state)
{
  std::map<std::string, double> joint_effort;
  const std::vector<std::string>& variable_name = robot_state->getVariableNames();
  for (auto& name : variable_name)
  {
    joint_effort[name.c_str()] = robot_state->getVariableEffort(name);
  }
  return joint_effort;
}

void set_joint_efforts(std::shared_ptr<moveit::core::RobotState>& robot_state,
                       std::map<std::string, double>& joint_efforts)
{
  for (const auto& item : joint_efforts)
  {
    robot_state->setVariableEffort(item.first, item.second);
  }
}

Eigen::VectorXd copy_joint_group_positions(std::shared_ptr<moveit::core::RobotState>& robot_state,
                                           const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  robot_state->copyJointGroupPositions(joint_model_group_name, values);
  return values;
}

Eigen::VectorXd copy_joint_group_velocities(std::shared_ptr<moveit::core::RobotState>& robot_state,
                                            const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  robot_state->copyJointGroupVelocities(joint_model_group_name, values);
  return values;
}

Eigen::VectorXd copy_joint_group_accelerations(std::shared_ptr<moveit::core::RobotState>& robot_state,
                                               const std::string& joint_model_group_name)
{
  Eigen::VectorXd values;
  robot_state->copyJointGroupAccelerations(joint_model_group_name, values);
  return values;
}

Eigen::MatrixXd get_jacobian(std::shared_ptr<moveit::core::RobotState>& robot_state,
                             const std::string& joint_model_group_name, const Eigen::Vector3d& reference_point_position)
{
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
  return robot_state->getJacobian(joint_model_group, reference_point_position);
}

Eigen::MatrixXd get_jacobian(std::shared_ptr<moveit::core::RobotState>& robot_state,
                             const std::string& joint_model_group_name, const std::string& link_model_name,
                             const Eigen::Vector3d& reference_point_position, bool use_quaternion_representation)
{
  Eigen::MatrixXd jacobian;
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
  const moveit::core::LinkModel* link_model = robot_state->getLinkModel(link_model_name);
  robot_state->getJacobian(joint_model_group, link_model, reference_point_position, jacobian,
                           use_quaternion_representation);
  return jacobian;
}

bool set_to_default_values(std::shared_ptr<moveit::core::RobotState>& robot_state,
                           const std::string& joint_model_group_name, const std::string& state_name)

{
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(joint_model_group_name);
  return robot_state->setToDefaultValues(joint_model_group, state_name);
}

void init_robot_state(py::module& m)
{
  py::module robot_state = m.def_submodule("robot_state");

  robot_state.def(
      "robotStateToRobotStateMsg",
      [](const moveit::core::RobotState& state, bool copy_attached_bodies) {
        moveit_msgs::msg::RobotState state_msg;
        moveit::core::robotStateToRobotStateMsg(state, state_msg, copy_attached_bodies);
        return state_msg;
      },
      py::arg("state"), py::arg("copy_attached_bodies") = true);

  py::class_<moveit::core::RobotState, std::shared_ptr<moveit::core::RobotState>>(robot_state, "RobotState",
                                                                                  DOC(moveit_core, RobotState))

      .def(py::init<const std::shared_ptr<const moveit::core::RobotModel>&>(), DOC(moveit_core, RobotState, RobotState))

      // Get underlying robot model, frame transformations and jacobian
      .def_property("robot_model", &moveit::core::RobotState::getRobotModel, nullptr,
                    py::return_value_policy::reference, DOC(moveit_core, RobotState, getRobotModel))

      .def_property("dirty", &moveit::core::RobotState::dirty, nullptr, DOC(moveit_core, RobotState, dirty))

      .def("get_frame_transform", &moveit_py::bind_robot_state::get_frame_transform, py::arg("frame_id"),
           py::return_value_policy::move, DOC(moveit_core, RobotState, getFrameTransform))

      .def("get_pose", &moveit_py::bind_robot_state::get_pose, py::arg("link_name"),
           DOC(moveit_core, RobotState, getGlobalLinkTransform))

      .def("get_jacobian",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const Eigen::Vector3d&>(
               &moveit_py::bind_robot_state::get_jacobian),
           py::arg("joint_model_group_name"), py::arg("reference_point_position"), py::return_value_policy::move,
           DOC(moveit_core, RobotState, getJacobian, 3))

      .def("get_jacobian",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const std::string&,
                             const Eigen::Vector3d&, bool>(&moveit_py::bind_robot_state::get_jacobian),
           py::arg("joint_model_group_name"), py::arg("link_name"), py::arg("reference_point_position"),
           py::arg("use_quaternion_representation") = false, py::return_value_policy::move,
           DOC(moveit_core, RobotState, getJacobian, 2))

      .def("set_joint_group_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupPositions),
           py::arg("joint_model_group_name"), py::arg("position_values"),
           DOC(moveit_core, RobotState, setJointGroupPositions))

      // peterdavidfagan: I am not sure if additional function names are better than having function parameters for joint setting.
      .def("set_joint_group_active_positions",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupActivePositions),
           py::arg("joint_model_group_name"), py::arg("position_values"),
           DOC(moveit_core, RobotState, setJointGroupActivePositions))

      .def("get_joint_group_positions", &moveit_py::bind_robot_state::copy_joint_group_positions,
           py::arg("joint_model_group_name"), DOC(moveit_core, RobotState, copyJointGroupPositions))

      .def("set_joint_group_velocities",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupVelocities),
           py::arg("joint_model_group_name"), py::arg("velocity_values"),
           DOC(moveit_core, RobotState, setJointGroupVelocities))

      .def("get_joint_group_velocities", &moveit_py::bind_robot_state::copy_joint_group_velocities,
           py::arg("joint_model_group_name"), DOC(moveit_core, RobotState, copyJointGroupVelocities))

      .def("set_joint_group_accelerations",
           py::overload_cast<const std::string&, const Eigen::VectorXd&>(
               &moveit::core::RobotState::setJointGroupAccelerations),
           py::arg("joint_model_group_name"), py::arg("acceleration_values"),
           DOC(moveit_core, RobotState, setJointGroupAccelerations))

      .def("get_joint_group_accelerations", &moveit_py::bind_robot_state::copy_joint_group_accelerations,
           py::arg("joint_model_group_name"), DOC(moveit_core, RobotState, copyJointGroupAccelerations))

      // Forward kinematics
      .def("get_global_link_transform", &moveit_py::bind_robot_state::get_global_link_transform, py::arg("link_name"),
           py::return_value_policy::move, DOC(moveit_core, RobotState, getGlobalLinkTransform))

      // Setting state from inverse kinematics
      .def(
          "set_from_ik",
          [](moveit::core::RobotState& robot_state, const std::string& group, const geometry_msgs::msg::Pose& pose,
             const std::string& tip, double timeout) {
            return robot_state.setFromIK(robot_state.getJointModelGroup(group), pose, tip, timeout);
          },
          py::arg("joint_model_group_name"), py::arg("geometry_pose"), py::arg("tip_name"), py::arg("timeout") = 0.0,
          DOC(moveit_core, RobotState, setFromIK, 3))

      // Setting entire state values
      .def("set_to_default_values", py::overload_cast<>(&moveit::core::RobotState::setToDefaultValues),
           DOC(moveit_core, RobotState, setToDefaultValues))

      .def("set_to_default_values",
           py::overload_cast<const moveit::core::JointModelGroup*, const std::string&>(
               &moveit::core::RobotState::setToDefaultValues),
           py::arg("joint_model_group"), py::arg("name"), DOC(moveit_core, RobotState, setToDefaultValues, 2))

      .def("set_to_default_values",
           py::overload_cast<std::shared_ptr<moveit::core::RobotState>&, const std::string&, const std::string&>(
               &moveit_py::bind_robot_state::set_to_default_values),
           py::arg("joint_model_group_name"), py::arg("name"), DOC(moveit_core, RobotState, setToDefaultValues, 2))

      .def("set_to_random_positions", py::overload_cast<>(&moveit::core::RobotState::setToRandomPositions),
           DOC(moveit_core, RobotState, setToRandomPositions))

      .def("set_to_random_positions",
           py::overload_cast<const moveit::core::JointModelGroup*>(&moveit::core::RobotState::setToRandomPositions),
           py::arg("joint_model_group"), DOC(moveit_core, RobotState, setToRandomPositions, 2))

      .def("clear_attached_bodies", py::overload_cast<>(&moveit::core::RobotState::clearAttachedBodies),
           DOC(moveit_core, RobotState, clearAttachedBodies, 3))

      .def("update", &moveit_py::bind_robot_state::update, py::arg("force") = false, py::arg("type") = "all",
           DOC(moveit_core, RobotState, update));
}
}  // namespace bind_robot_state
}  // namespace moveit_py
