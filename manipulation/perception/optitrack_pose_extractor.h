#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "optitrack/optitrack_data_descriptions_t.hpp"
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace perception {

/**
 * Gets the pose of an Optitrack rigid body.
 * @returns X_OB, the pose of the body `B` in the optitrack frame `O`.
 */
DRAKE_DEPRECATED("2022-02-01", "Possibly use OptitrackReceiver instead.")
Isometry3<double> ExtractOptitrackPose(
    const optitrack::optitrack_rigid_body_t& message);

/**
 * Extracts poses of all objects from an Optitrack message.
 * @returns Mapping from object ID to pose.
 */
DRAKE_DEPRECATED("2022-02-01", "This function is being removed.")
std::map<int, Isometry3<double>> ExtractOptitrackPoses(
    const optitrack::optitrack_frame_t& frame);

/**
 * Gets a rigid body from an optitrack frame message given an object ID.
 * @param message Optitrack message.
 * @param object_id ID to be searched for in the frame message.
 * @returns Rigid body object, or `nullopt` if not found.
 */
DRAKE_DEPRECATED("2022-02-01", "This function is being removed.")
std::optional<optitrack::optitrack_rigid_body_t> FindOptitrackBody(
      const optitrack::optitrack_frame_t& message, int object_id);

/**
 * Gets the object ID from an Optitrack description message.
 * @param message Description message.
 * @returns Object ID if found, or `nullopt` if not found.
 */
DRAKE_DEPRECATED("2022-02-01", "This function is being removed.")
std::optional<int> FindOptitrackObjectId(
    const optitrack::optitrack_data_descriptions_t& message,
    const std::string& object_name);

/**
 * Extracts and provides an output of the pose of a desired object as an
 * Eigen::Isometry3d from an Optitrack LCM OPTITRACK_FRAME_T message, the
 * pose transformed to a desired coordinate frame.
 *
 * @ingroup manipulation_systems
 */
class DRAKE_DEPRECATED("2022-02-01", "Use OptitrackReceiver instead.")
OptitrackPoseExtractor : public systems::LeafSystem<double> {
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
