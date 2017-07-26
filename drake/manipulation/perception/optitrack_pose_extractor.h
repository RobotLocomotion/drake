#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace perception {

/**
 * Extracts and provides an output of the pose of a desired object (A vector of
 * dimension 7 - Cartesian position as the first 3 and the orientation
 * in quaternions as the next 4) from an Optitrack LCM OPTITRACK_FRAME_T
 * message, transformed to a desired coordinate frame.
 */
class OptitrackPoseExtractor : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackPoseExtractor)
  /**
   * Constructs the OptitrackPoseExtractor.
   * @param object_id : An ID of the object being tracked. This ID must
   * correspond to the those present within the OPTITRACK_FRAME_T message or
   * else a runtime exception is thrown.
   * @param world_X_optitrack : The transformation from world frame (Drake
   * frame) to the optitrack frame.
   * @param optitrack_lcm_status_period : The discrete update period of the
   * OptitrackPoseExtractor. It should be set based on the period of incoming
   * optitrack messages.
   */
  OptitrackPoseExtractor(unsigned int object_id,
                         const Isometry3<double>& world_X_optitrack,
                         double optitrack_lcm_status_period);

  const systems::OutputPort<double>& get_measured_pose_output_port() const {
    return this->get_output_port(measured_pose_output_port_);
  }

 protected:
  void OutputMeasuredPose(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;

  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
      systems::DiscreteValues<double>* discrete_state) const override;

 private:
  const unsigned int object_id_{0};
  const int measured_pose_output_port_{-1};
  // Pose of the optitrack frame O in the world frame W.
  const Isometry3<double> X_WOp_;
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
