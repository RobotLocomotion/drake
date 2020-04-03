#pragma once

#include <memory>
#include <vector>

#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace perception {
/**
 * This class accepts the pose of a rigid body (composed by a
 * Eigen::Isometry3d) and returns a smoothed pose by performing either the first
 * or both of these processes :
 *  i. Rejecting outliers on the basis of user-defined linear/angular velocity
 *  thresholds on consecutive pose data values.
 *  ii. Moving average smoothing of the resulting data within a specified
 *  window size.
 *  Note on quaternion averaging :
 *  While a "correct" quaternion averaging algorithm requires averaging the
 *  corresponding attitudes, this class implements a simplification based on
 *  the version described in
 * http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors
 *  and in the introduction of Markley et al.
 *  References:
 *  L. Markley, Y. Cheng, J. L. Crassidis, and Y. Oshman, "Quaternion
 * Averaging",
 *  NASA Technical note, available to download at
 *  https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 *
 *  @ingroup manipulation_systems
 */
class PoseSmoother : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSmoother)

  /**
   * Constructs the pose smoother with or without averaging - i.e. performs
   * outlier rejection and smoothing of the input pose. Smoothing is disabled
   * for a window size lesser than 1.
   * @param desired_max_linear_velocity Upper threshold on linear velocity
   * (m/sec).
   * @param desired_max_angular_velocity Upper threshold on angular velocity
   * (rad/sec).
   * @param period_sec The period for the internal update (sec).
   * This must be set to a value greater than 0.
   * @param filter_window_size Window size for the moving average smoothing.
   * Must be set to a value greater than 1 to enable averaging (smoothing).
   */
  PoseSmoother(double desired_max_linear_velocity,
               double desired_max_angular_velocity,
               double period_sec, int filter_window_size);

  const systems::OutputPort<double>& get_smoothed_pose_output_port() const {
    return this->get_output_port(smoothed_pose_output_port_);
  }

  const systems::OutputPort<double>& get_smoothed_velocity_output_port() const {
    return this->get_output_port(smoothed_velocity_output_port_);
  }

 private:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& event,
      systems::State<double>* state) const override;

  void OutputSmoothedPose(const systems::Context<double>& context,
                          Eigen::Isometry3d* output) const;

  void OutputSmoothedVelocity(const systems::Context<double>& context,
                              Vector6<double>* output) const;

 private:
  const int smoothed_pose_output_port_{0};
  const int smoothed_velocity_output_port_{0};
  const double max_linear_velocity_{0.0};
  const double max_angular_velocity_{0.0};
  const bool is_filter_enabled_{false};
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
