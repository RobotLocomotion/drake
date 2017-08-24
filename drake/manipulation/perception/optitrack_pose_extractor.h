#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace perception {

/**
 * Extracts and provides an output of the pose of a desired object as an
 * Eigen::Isometry3d from an Optitrack LCM OPTITRACK_FRAME_T message, the
 * pose transformed to a desired coordinate frame.
 */
class OptitrackPoseExtractor : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackPoseExtractor)
  /**
   * Constructs the OptitrackPoseExtractor.
   * @param object_id An ID of the object being tracked. This ID must
   * correspond to the those present within the OPTITRACK_FRAME_T message or
   * else a runtime exception is thrown.
   * @param X_WO The pose of the optitrack frame O in the World frame W.
   * @param optitrack_lcm_status_period The discrete update period of the
   * OptitrackPoseExtractor. It should be set based on the period of incoming
   * optitrack messages.
   */
  OptitrackPoseExtractor(int object_id, const Isometry3<double>& X_WO,
                         double optitrack_lcm_status_period);

  const systems::OutputPort<double>& get_measured_pose_output_port() const {
    return this->get_output_port(measured_pose_output_port_);
  }

 private:
  void DoCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& event,
      systems::State<double>* state) const override;

  // The Calc() method for the measured_pose_output_port.
  void OutputMeasuredPose(const systems::Context<double>& context,
                          Isometry3<double>* output) const;

  const int object_id_{0};
  const int measured_pose_output_port_{-1};
  // Pose of the optitrack frame O in the world frame W.
  const Isometry3<double> X_WO_;
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
