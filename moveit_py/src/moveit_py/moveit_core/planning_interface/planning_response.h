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
 *   * Neither the name of PickNik Inc. nor te names of its
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

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit_msgs/msg/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.h>
#include <serialize_ros_msg.h>
#include <moveit/planning_interface/planning_response.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_planning_interface
{

std::shared_ptr<robot_trajectory::RobotTrajectory>
get_motion_plan_response_trajectory(std::shared_ptr<planning_interface::MotionPlanResponse>& response);

py::object get_motion_plan_response_start_state(std::shared_ptr<planning_interface::MotionPlanResponse>& response);

py::object get_motion_plan_response_error_code(std::shared_ptr<planning_interface::MotionPlanResponse>& response);

double get_motion_plan_response_planning_time(std::shared_ptr<planning_interface::MotionPlanResponse>& response);

std::string get_motion_plan_response_planner_id(std::shared_ptr<planning_interface::MotionPlanResponse>& response);

void init_motion_plan_response(py::module& m);
}  // namespace bind_planning_interface
}  // namespace moveit_py
