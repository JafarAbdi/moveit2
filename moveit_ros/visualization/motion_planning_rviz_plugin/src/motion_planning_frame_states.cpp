/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Mario Prats, Ioan Sucan */
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>

#include <QMessageBox>
#include <QInputDialog>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame_states");

void MotionPlanningFrame::populateRobotStatesList()
{
  ui_->list_states->clear();
  for (std::pair<const std::string, moveit_msgs::msg::RobotState>& robot_state : robot_states_)
  {
    QListWidgetItem* item = new QListWidgetItem(QString(robot_state.first.c_str()));
    ui_->list_states->addItem(item);
  }
}

void MotionPlanningFrame::loadStateButtonClicked()
{
  throw std::runtime_error("loadStateButtonClicked not implemented");
}

void MotionPlanningFrame::loadStoredStates(const std::string& /*pattern*/)
{
  throw std::runtime_error("loadStoredStates not implemented");
}

void MotionPlanningFrame::saveRobotStateButtonClicked(const moveit::core::RobotState& /*state*/)
{
  throw std::runtime_error("saveRobotStateButtonClicked not implemented");
}

void MotionPlanningFrame::saveStartStateButtonClicked()
{
  saveRobotStateButtonClicked(*planning_display_->getQueryStartState());
}

void MotionPlanningFrame::saveGoalStateButtonClicked()
{
  saveRobotStateButtonClicked(*planning_display_->getQueryGoalState());
}

void MotionPlanningFrame::setAsStartStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    moveit::core::RobotState robot_state(*planning_display_->getQueryStartState());
    moveit::core::robotStateMsgToRobotState(robot_states_[item->text().toStdString()], robot_state);
    planning_display_->setQueryStartState(robot_state);
  }
}

void MotionPlanningFrame::setAsGoalStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    moveit::core::RobotState robot_state(*planning_display_->getQueryGoalState());
    moveit::core::robotStateMsgToRobotState(robot_states_[item->text().toStdString()], robot_state);
    planning_display_->setQueryGoalState(robot_state);
  }
}

void MotionPlanningFrame::removeStateButtonClicked()
{
  throw std::runtime_error("removeStateButtonClicked not implemented");
}

void MotionPlanningFrame::clearStatesButtonClicked()
{
  QMessageBox msg_box;
  msg_box.setText("Clear all stored robot states (from memory, not from the database)?");
  msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
  msg_box.setDefaultButton(QMessageBox::Yes);
  int ret = msg_box.exec();
  switch (ret)
  {
    case QMessageBox::Yes:
    {
      robot_states_.clear();
      populateRobotStatesList();
    }
    break;
  }
  return;
}

}  // namespace moveit_rviz_plugin
