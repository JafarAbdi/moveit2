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

#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame_context");

void MotionPlanningFrame::databaseConnectButtonClicked()
{
  throw std::runtime_error("databaseConnectButtonClicked: not implemented");
}

void MotionPlanningFrame::planningPipelineIndexChanged(int index)
{
  // Refresh planner interface description for selected pipeline
  if (index >= 0 && static_cast<size_t>(index) < planner_descriptions_.size())
  {
    // Set the selected pipeline id
    if (move_group_)
      move_group_->setPlanningPipelineId(planner_descriptions_[index].pipeline_id);

    populatePlannerDescription(planner_descriptions_[index]);
  }
}

void MotionPlanningFrame::planningAlgorithmIndexChanged(int index)
{
  std::string planner_id = ui_->planning_algorithm_combo_box->itemText(index).toStdString();
  if (index <= 0)
    planner_id = "";

  ui_->planner_param_treeview->setPlannerId(planner_id);

  if (move_group_)
    move_group_->setPlannerId(planner_id);
}

void MotionPlanningFrame::resetDbButtonClicked()
{
  if (QMessageBox::warning(this, "Data about to be deleted",
                           "The following dialog will allow you to drop a MoveIt "
                           "Warehouse database. Are you sure you want to continue?",
                           QMessageBox::Yes | QMessageBox::No) == QMessageBox::No)
    return;

  QStringList dbs;
  dbs.append("Planning Scenes");
  dbs.append("Constraints");
  dbs.append("Robot States");

  bool ok = false;
  QString response =
      QInputDialog::getItem(this, tr("Select Database"), tr("Choose the database to reset:"), dbs, 2, false, &ok);
  if (!ok)
    return;

  if (QMessageBox::critical(
          this, "Data about to be deleted",
          QString("All data in database '").append(response).append("'. Are you sure you want to continue?"),
          QMessageBox::Yes | QMessageBox::No) == QMessageBox::No)
    return;

  planning_display_->addBackgroundJob([this, db = response.toStdString()] { computeResetDbButtonClicked(db); },
                                      "reset database");
}

void MotionPlanningFrame::computeDatabaseConnectButtonClicked()
{
  throw std::runtime_error("computeDatabaseConnectButtonClicked: not implemented");
}

void MotionPlanningFrame::computeDatabaseConnectButtonClickedHelper(int /*mode*/)
{
  throw std::runtime_error("computeDatabaseConnectButtonClickedHelper: not implemented");
}

void MotionPlanningFrame::computeResetDbButtonClicked(const std::string& /*db*/)
{
  throw std::runtime_error("computeResetDbButtonClicked: not implemented");
}
}  // namespace moveit_rviz_plugin
