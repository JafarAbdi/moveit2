/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of KU Leuven nor the names of its
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

/* Author: Jeroen De Maeyer, Boston Cleek */

#include <algorithm>
#include <iterator>

#include <moveit/ompl_interface/detail/ompl_constraints.h>

#include <tf2_eigen/tf2_eigen.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_planners_ompl.ompl_constraints");

namespace ompl_interface
{
static const double MAX_QUATERNION_NORM_ERROR = 1e-9;

Bounds::Bounds() : size_(0)
{
}

Bounds::Bounds(const std::vector<double>& lower, const std::vector<double>& upper)
  : lower_(lower), upper_(upper), size_(lower.size())
{
  // how to report this in release mode??
  assert(lower_.size() == upper_.size());
}

Eigen::VectorXd Bounds::penalty(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert((long)lower_.size() == x.size());
  Eigen::VectorXd penalty(x.size());

  for (unsigned int i = 0; i < x.size(); ++i)
  {
    if (x[i] < lower_.at(i))
    {
      penalty[i] = lower_.at(i) - x[i];
    }
    else if (x[i] > upper_.at(i))
    {
      penalty[i] = x[i] - upper_.at(i);
    }
    else
    {
      penalty[i] = 0.0;
    }
  }
  return penalty;
}

Eigen::VectorXd Bounds::derivative(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  assert((long)lower_.size() == x.size());
  Eigen::VectorXd derivative(x.size());

  for (unsigned int i = 0; i < x.size(); ++i)
  {
    if (x[i] < lower_.at(i))
    {
      derivative[i] = -1.0;
    }
    else if (x[i] > upper_.at(i))
    {
      derivative[i] = 1.0;
    }
    else
    {
      derivative[i] = 0.0;
    }
  }
  return derivative;
}

std::size_t Bounds::size() const
{
  return size_;
}

std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bounds)
{
  os << "Bounds:\n";
  for (std::size_t i{ 0 }; i < bounds.size(); ++i)
  {
    os << "( " << bounds.lower_[i] << ", " << bounds.upper_[i] << " )\n";
  }
  return os;
}

/****************************
 * Base class for constraints
 * **************************/
BaseConstraint::BaseConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_cons_)
  : ompl::base::Constraint(num_dofs, num_cons_)
  , state_storage_(robot_model)
  , joint_model_group_(robot_model->getJointModelGroup(group))

{
}

void BaseConstraint::init(const moveit_msgs::msg::Constraints& constraints)
{
  parseConstraintMsg(constraints);
}

void BaseConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::VectorXd current_values = calcError(joint_values);
  out = bounds_.penalty(current_values);
}

void BaseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              Eigen::Ref<Eigen::MatrixXd> out) const
{
  const Eigen::VectorXd constraint_error = calcError(joint_values);
  const Eigen::VectorXd constraint_derivative = bounds_.derivative(constraint_error);
  const Eigen::MatrixXd robot_jacobian = calcErrorJacobian(joint_values);
  for (std::size_t i = 0; i < bounds_.size(); ++i)
  {
    out.row(i) = constraint_derivative[i] * robot_jacobian.row(i);
  }
}

Eigen::Isometry3d BaseConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state->getGlobalLinkTransform(link_name_);
}

Eigen::MatrixXd BaseConstraint::robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  moveit::core::RobotState* robot_state = state_storage_.getStateStorage();
  robot_state->setJointGroupPositions(joint_model_group_, joint_values);
  Eigen::MatrixXd jacobian;
  // return value (success) not used, could return a garbage jacobian.
  robot_state->getJacobian(joint_model_group_, joint_model_group_->getLinkModel(link_name_),
                           Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);
  return jacobian;
}

Eigen::VectorXd BaseConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& /*x*/) const
{
  RCLCPP_WARN_STREAM(LOGGER,
                     "BaseConstraint: Constraint method calcError was not overridden, so it should not be used.");
  return Eigen::VectorXd::Zero(getCoDimension());
}

Eigen::MatrixXd BaseConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& /*x*/) const
{
  RCLCPP_WARN_STREAM(
      LOGGER, "BaseConstraint: Constraint method calcErrorJacobian was not overridden, so it should not be used.");
  return Eigen::MatrixXd::Zero(getCoDimension(), n_);
}

/******************************************
 * Position constraints
 * ****************************************/
BoxConstraint::BoxConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                             const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void BoxConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  assert(bounds_.size() == 0);
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));

  // extract target position and orientation
  geometry_msgs::msg::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;

  target_position_ << position.x, position.y, position.z;

  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  link_name_ = constraints.position_constraints.at(0).link_name;
}

Eigen::VectorXd BoxConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
}

Eigen::MatrixXd BoxConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return target_orientation_.matrix().transpose() * robotGeometricJacobian(x).topRows(3);
}

/******************************************
 * Equality constraints
 * ****************************************/
EqualityPositionConstraint::EqualityPositionConstraint(const moveit::core::RobotModelConstPtr& robot_model,
                                                       const std::string& group, const unsigned int num_dofs)
  : BaseConstraint(robot_model, group, num_dofs)
{
}

void EqualityPositionConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  const auto dims = constraints.position_constraints.at(0).constraint_region.primitives.at(0).dimensions;

  is_dim_constrained_ = { false, false, false };
  for (std::size_t i = 0; i < dims.size(); ++i)
  {
    if (dims.at(i) < EQUALITY_CONSTRAINT_THRESHOLD)
    {
      if (dims.at(i) < getTolerance())
      {
        RCLCPP_ERROR_STREAM(
            LOGGER,
            "Dimension: " << i
                          << " of position constraint is smaller than the tolerance used to evaluate the constraints. "
                             "This will make all states invalid and planning will fail. Please use a value between: "
                          << getTolerance() << " and " << EQUALITY_CONSTRAINT_THRESHOLD);
      }

      is_dim_constrained_.at(i) = true;
    }
  }

  // extract target position and orientation
  geometry_msgs::msg::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;

  target_position_ << position.x, position.y, position.z;

  tf2::fromMsg(constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).orientation,
               target_orientation_);

  link_name_ = constraints.position_constraints.at(0).link_name;
}

void EqualityPositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::VectorXd> out) const
{
  Eigen::Vector3d error =
      target_orientation_.matrix().transpose() * (forwardKinematics(joint_values).translation() - target_position_);
  for (std::size_t dim = 0; dim < 3; ++dim)
  {
    if (is_dim_constrained_.at(dim))
    {
      out[dim] = error[dim];  // equality constraint dimension
    }
    else
    {
      out[dim] = 0.0;  // unbounded dimension
    }
  }
}

void EqualityPositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                          Eigen::Ref<Eigen::MatrixXd> out) const
{
  out.setZero();
  Eigen::MatrixXd jac = target_orientation_.matrix().transpose() * robotGeometricJacobian(joint_values).topRows(3);
  for (std::size_t dim = 0; dim < 3; ++dim)
  {
    if (is_dim_constrained_.at(dim))
    {
      out.row(dim) = jac.row(dim);  // equality constraint dimension
    }
  }
}

/******************************************
 * Orientation constraints
 * ****************************************/
void OrientationConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints)
{
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));

  tf2::fromMsg(constraints.orientation_constraints.at(0).orientation, target_orientation_);

  link_name_ = constraints.orientation_constraints.at(0).link_name;
}

Eigen::VectorXd OrientationConstraint::calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa(orientation_difference);
  return aa.axis() * aa.angle();
}

Eigen::MatrixXd OrientationConstraint::calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  Eigen::Matrix3d orientation_difference = forwardKinematics(x).linear().transpose() * target_orientation_;
  Eigen::AngleAxisd aa{ orientation_difference };
  return -angularVelocityToAngleAxis(aa.angle(), aa.axis()) * robotGeometricJacobian(x).bottomRows(3);
}

/************************************
 * MoveIt constraint message parsing
 * **********************************/
Bounds positionConstraintMsgToBoundVector(const moveit_msgs::msg::PositionConstraint& pos_con)
{
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
    {
      dim = std::numeric_limits<double>::infinity();
    }
  }

  return { { -dims.at(0) / 2.0, -dims.at(1) / 2.0, -dims.at(2) / 2.0 },
           { dims.at(0) / 2.0, dims.at(1) / 2.0, dims.at(2) / 2.0 } };
}

Bounds orientationConstraintMsgToBoundVector(const moveit_msgs::msg::OrientationConstraint& ori_con)
{
  std::vector<double> dims = { ori_con.absolute_x_axis_tolerance, ori_con.absolute_y_axis_tolerance,
                               ori_con.absolute_z_axis_tolerance };

  // dimension of -1 signifies unconstrained parameter, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = std::numeric_limits<double>::infinity();
  }
  return { { -dims[0], -dims[1], -dims[2] }, { dims[0], dims[1], dims[2] } };
}

/******************************************
 * OMPL Constraints Factory
 * ****************************************/
ompl::base::ConstraintPtr createOMPLConstraints(const moveit::core::RobotModelConstPtr& robot_model,
                                                const std::string& group,
                                                const moveit_msgs::msg::Constraints& constraints)
{
  // TODO(bostoncleek): does this reach the end w/o a return ?

  const std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  const std::size_t num_pos_con = constraints.position_constraints.size();
  const std::size_t num_ori_con = constraints.orientation_constraints.size();

  // This factory method contains template code to support position and/or orientation constraints.
  // If the specified constraints are invalid, a nullptr is returned.
  std::vector<ompl::base::ConstraintPtr> ompl_constraints;
  if (num_pos_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single position constraint is supported. Using the first one.");
  }
  if (num_ori_con > 1)
  {
    RCLCPP_WARN(LOGGER, "Only a single orientation constraint is supported. Using the first one.");
  }
  if (num_pos_con > 0)
  {
    BaseConstraintPtr pos_con;
    if (constraints.name == "use_equality_constraints")
    {
      pos_con = std::make_shared<EqualityPositionConstraint>(robot_model, group, num_dofs);
    }
    else
    {
      pos_con = std::make_shared<BoxConstraint>(robot_model, group, num_dofs);
    }
    pos_con->init(constraints);
    ompl_constraints.emplace_back(pos_con);
  }
  if (num_ori_con > 0)
  {
    auto ori_con = std::make_shared<OrientationConstraint>(robot_model, group, num_dofs);
    ori_con->init(constraints);
    ompl_constraints.emplace_back(ori_con);
  }
  if (num_pos_con < 1 && num_ori_con < 1)
  {
    RCLCPP_ERROR(LOGGER, "No path constraints found in planning request.");
    return nullptr;
  }
  return std::make_shared<ompl::base::ConstraintIntersection>(num_dofs, ompl_constraints);
}

/******************************************
 * Toolpath Constraint
 * ****************************************/
ToolPathConstraint::ToolPathConstraint(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group,
                                       unsigned int num_dofs, const unsigned int num_cons, double step)
  : BaseConstraint(robot_model, group, num_dofs, num_cons), step_(step)
{
  pose_nn_.setDistanceFunction([this](const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) {
    return poseDistance(pose1, pose2);
  });
}


void ToolPathConstraint::parseConstraintMsg(const moveit_msgs::msg::Constraints& constraints) override
{
  // TODO: Is this needed? Can this be done using setPath()?
}


void ToolPathConstraint::setPath(const moveit_msgs::msg::CartesianTrajectory& msg)
{
  // Clear previous path
  pose_nn_.clear();

  // Interpolate along waypoints and insert into pose_nn_;
  for (unsigned int i = 0; i < msg.points.size() - 1; i++)
  {
    // Insert waypoints
    pose_nn_.add(msg.points.at(i).point.pose);

    // TODO: Consider arc lenth between the poses to determine the number of interpolation steps
    // Linearly determine the number of steps
    const auto num_steps = static_cast<unsigned int>(
        euclideanDistance(msg.points.at(i).point.pose, msg.points.at(i + 1).point.pose) / step_);

    // Insert interpolated poses
    auto delta = step_;
    for (unsigned int j = 0; j < num_steps - 1; j++)
    {
      const auto pose = poseInterpolator(msg.points.at(i).point.pose, msg.points.at(i + 1).point.pose, delta);
      pose_nn_.add(pose);
      delta += step_;
    }
  }

  // Insert last waypoint
  pose_nn_.add(msg.points.at(msg.points.size() - 1).point.pose);
}

void ToolPathConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  // TODO: try setting num_dim = 1 in BaseConstraint and using poseDistance as the constraint function
  // Ideal: F(q_actual) - F(q_nearest_pose) = 0

  // Actual end-effector pose
  const Eigen::Isometry3d eef_pose_eigen = forwardKinematics(joint_values);
  const Eigen::Vector3d trans = eef_pose_eigen.translation();
  const Eigen::Quaterniond eef_quat(eef_pose_eigen.rotation());
  const Eigen::AngleAxisd eef_aa(eef_quat);
  const Eigen::Vector3d eef_aa_vec = eef_aa.axis() * eef_aa.angle();

  geometry_msgs::msg::Pose eef_pose;
  eef_pose.position.x = trans.x();
  eef_pose.position.y = trans.y();
  eef_pose.position.z = trans.z();

  eef_pose.orientation.w = eef_quat.w();
  eef_pose.orientation.x = eef_quat.x();
  eef_pose.orientation.y = eef_quat.y();
  eef_pose.orientation.z = eef_quat.z();

  // Nearest end-effector pose in trajectory
  const geometry_msgs::msg::Pose eef_nearest_pose = pose_nn_.nearest(eef_pose);
  const Eigen::Quaterniond eef_nearest_quat(eef_nearest_pose.orientation.w, eef_nearest_pose.orientation.x,
                                            eef_nearest_pose.orientation.y, eef_nearest_pose.orientation.z);
  const Eigen::AngleAxisd eef_nearest_aa(eef_quat);
  const Eigen::Vector3d eef_nearest_aa_vec = eef_nearest_aa.axis() * eef_nearest_aa.angle();

  // should be allocated to size coDim, by default this is 6
  out[0] = eef_pose.position.x - eef_nearest_pose.position.x;
  out[1] = eef_pose.position.y - eef_nearest_pose.position.y;
  out[2] = eef_pose.position.z - eef_nearest_pose.position.z;

  out[3] = eef_aa_vec(0) - eef_nearest_aa_vec(0);
  out[4] = eef_aa_vec(1) - eef_nearest_aa_vec(1);
  out[5] = eef_aa_vec(2) - eef_nearest_aa_vec(2);
}

double poseDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
{
  return euclideanDistance(pose1, pose2) + arcLength(pose1, pose2);
}

double euclideanDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
{
  const auto dx = pose1.position.x - pose2.position.x;
  const auto dy = pose1.position.y - pose2.position.y;
  const auto dz = pose1.position.z - pose2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double arcLength(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
{
  // Distance between two quaternions
  // https://math.stackexchange.com/questions/90081/quaternion-distance
  // https://github.com/ompl/ompl/blob/main/src/ompl/base/spaces/src/SO3StateSpace.cpp#L254-L262
  // dot product between q1 and q2
  auto dq = std::fabs(pose1.orientation.w * pose2.orientation.w + pose1.orientation.x * pose2.orientation.x +
                      pose1.orientation.y * pose2.orientation.y + pose1.orientation.z * pose2.orientation.z);

  if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
  {
    return 0.0;
  }

  return std::acos(dq);
}

geometry_msgs::msg::Pose poseInterpolator(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2,
                                          double step)
{
  // Pose interpolation
  // https://answers.ros.org/question/299691/interpolation-for-between-two-poses/

  const Eigen::Vector3d t1(pose1.position.x, pose1.position.y, pose1.position.z);
  const Eigen::Vector3d t2(pose2.position.x, pose2.position.y, pose2.position.z);

  const Eigen::Quaterniond q1(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
  const Eigen::Quaterniond q2(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);

  const Eigen::Vector3d interp_translation = (1.0 - step) * t1 + step * t2;
  const Eigen::Quaterniond interp_orientation = q1.slerp(step, q2);

  geometry_msgs::msg::Pose interp_pose;
  interp_pose.position.x = interp_translation.x();
  interp_pose.position.y = interp_translation.y();
  interp_pose.position.z = interp_translation.z();

  interp_pose.orientation.w = interp_orientation.w();
  interp_pose.orientation.x = interp_orientation.x();
  interp_pose.orientation.y = interp_orientation.y();
  interp_pose.orientation.z = interp_orientation.z();

  return interp_pose;
}
}  // namespace ompl_interface
