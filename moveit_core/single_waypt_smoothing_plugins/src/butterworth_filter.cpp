#include <moveit/single_waypt_smoothing_plugins/butterworth_filter.h>

namespace single_waypt_smoothing_plugins
{
namespace
{
constexpr double EPSILON = 1e-9;
}

ButterworthFilter::ButterworthFilter(double low_pass_filter_coeff)
  : previous_measurements_{ 0., 0. }
  , previous_filtered_measurement_(0.)
  , scale_term_(1. / (1. + low_pass_filter_coeff))
  , feedback_term_(1. - low_pass_filter_coeff)
{
  // guarantee this doesn't change because the logic below depends on this length implicity
  static_assert(ButterworthFilter::FILTER_LENGTH == 2,
                "single_waypt_smoothing_plugins::ButterworthFilter::FILTER_LENGTH should be 2");

  if (std::isinf(feedback_term_))
    throw std::length_error("single_waypt_smoothing_plugins::ButterworthFilter: infinite feedback_term_");

  if (std::isinf(scale_term_))
    throw std::length_error("single_waypt_smoothing_plugins::ButterworthFilter: infinite scale_term_");

  if (low_pass_filter_coeff < 1)
    throw std::length_error(
        "single_waypt_smoothing_plugins::ButterworthFilter: Filter coefficient < 1. makes the lowpass filter unstable");

  if (std::abs(feedback_term_) < EPSILON)
    throw std::length_error(
        "single_waypt_smoothing_plugins::ButterworthFilter: Filter coefficient value resulted in feedback term of 0");
}

double ButterworthFilter::filter(double new_measurement)
{
  // Push in the new measurement
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement;

  double new_filtered_measurement = scale_term_ * (previous_measurements_[1] + previous_measurements_[0] -
                                                   feedback_term_ * previous_filtered_measurement_);

  // Store the new filtered measurement
  previous_filtered_measurement_ = new_filtered_measurement;

  return new_filtered_measurement;
}

void ButterworthFilter::reset(const double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;
  previous_filtered_measurement_ = data;
}

bool ButterworthFilterPlugin::initialize(rclcpp::Node::SharedPtr node, const moveit::core::JointModelGroup& /*unused*/,
                                         size_t num_dof, double /*unused*/)
{
  node_ = node;
  num_dof_ = num_dof;

  for (std::size_t i = 0; i < num_dof_; ++i)
  {
    // Low-pass filters for the joint positions
    // TODO(andyz): read the parameter
    position_filters_.emplace_back(1.5);
  }
  return true;
};

bool ButterworthFilterPlugin::doSmoothing(std::vector<double>& position_vector, std::vector<double>& /*unused*/)
{
  if (position_vector.size() != position_filters_.size())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Position vector to be smoothed does not have the right length.");
    return false;
  }
  for (size_t i = 0; i < position_vector.size(); ++i)
  {
    // Lowpass filter the position command
    position_vector[i] = position_filters_.at(i).filter(position_vector[i]);
  }
  return true;
};

bool ButterworthFilterPlugin::reset(const std::vector<double>& joint_positions)
{
  if (joint_positions.size() != position_filters_.size())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Position vector to be smoothed does not have the right length.");
    return false;
  }
  for (size_t joint_idx = 0; joint_idx < joint_positions.size(); ++joint_idx)
  {
    position_filters_.at(joint_idx).reset(joint_positions.at(joint_idx));
  }
  return true;
};

}  // namespace single_waypt_smoothing_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(single_waypt_smoothing_plugins::ButterworthFilterPlugin,
                       single_waypt_smoothing_plugins::SmoothingBaseClass)
