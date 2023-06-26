/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>

#include <interactive_markers/tools.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

#include <boost/math/constants/constants.hpp>

#include <memory>

namespace moveit_rviz_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame_scenes");

void MotionPlanningFrame::saveSceneButtonClicked()
{
  throw std::runtime_error("saveSceneButtonClicked not implemented");
}

void MotionPlanningFrame::planningSceneItemClicked()
{
  checkPlanningSceneTreeEnabledButtons();
}

void MotionPlanningFrame::saveQueryButtonClicked()
{
  throw std::runtime_error("saveQueryButtonClicked not implemented");
}

void MotionPlanningFrame::deleteSceneButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeDeleteSceneButtonClicked(); }, "delete scene");
}

void MotionPlanningFrame::deleteQueryButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeDeleteQueryButtonClicked(); }, "delete query");
}

void MotionPlanningFrame::loadSceneButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeLoadSceneButtonClicked(); }, "load scene");
}

void MotionPlanningFrame::loadQueryButtonClicked()
{
  planning_display_->addBackgroundJob([this] { computeLoadQueryButtonClicked(); }, "load query");
}

void MotionPlanningFrame::warehouseItemNameChanged(QTreeWidgetItem* /*item*/, int /*column*/)
{
  throw std::runtime_error("warehouseItemNameChanged not implemented");
}

void MotionPlanningFrame::populatePlanningSceneTreeView()
{
  throw std::runtime_error("populatePlanningSceneTreeView not implemented");
}
}  // namespace moveit_rviz_plugin
