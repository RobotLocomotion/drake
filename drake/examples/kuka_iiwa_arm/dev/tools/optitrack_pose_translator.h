# pragma once

#include <string>
#include <vector>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

//TODO(naveenoid) : This method should ideally not hardcode the computations but
// utilise settings from some param file of some sort.
/**
 * The default Transform between the Optitrack and world frames.
 *
 */
Isometry3<double> DefaultWorldOptitrackTransform();

/**
 * A translator to extract the pose of a single object as an
 * `Eigen::Isometry3d` in the world frame from within an optitrack LCM
 * `optitrack::optitrack_frame_t` message. This class can also be setup to
 * use a `MovingAverageFilter` to smoothen the position component of the
 * object poses.
 */
class OptitrackPoseTranslator {
 public:
  /**
   * Constructs the `OptitrackPoseTranslator` with the position filter
   * disabled.
   * @param object_id: The ID of the object whose pose is to be translated. The
   * ID must correspond to one of the models being tracked by optitrack and
   * exist within every `optitrack::optitrack_frame_t` message.
   * @param world_X_optitrack: The transform between the world frame within
   * Drake and the optitrack frame.
   * Note that a default argument using a hard-coded method can be used to
   * generate this transform to corresponding to experimental measurements.
   */
  OptitrackPoseTranslator(
      int object_id,
      const Isometry3<double>& world_X_optitrack =
      DefaultWorldOptitrackTransform());

  /**
   * Constructs the `OptitrackPoseTranslator` with the position filter
   * enabled.
   * @param object_id: The ID of the object whose pose is to be translated. The
   * ID must correspond to one of the models being tracked by optitrack and
   * exist within every `optitrack::optitrack_frame_t` message.
   * @param filter_window_size window size for the position filter.
   * Note:
   * 1. The orientation component of the returned pose is not filtered.
   * 2. A default argument using a hard-coded method can be used to
   * generate this transform to corresponding to experimental measurements.
   */
  OptitrackPoseTranslator(
      int object_id, int filter_window_size,
      const Isometry3<double>& world_X_optitrack =
      DefaultWorldOptitrackTransform());

  /**
   * Extracts and translates the pose of the object represented by `object_id`.
   * @param optitrack_msg: the LCM message that is to be translated.
   * @return The pose of the tracked object. The position component is
   * filtered using a moving average filter if the `OptitrackPoseTranslator`
   * has been constructed without a filter window being specified.
   */
  Isometry3<double> TranslatePose(
      const optitrack::optitrack_frame_t* optitrack_msg);

private :
  unsigned int object_index_{0};
  const Isometry3<double> world_X_optitrack_;
  std::unique_ptr<MovingAverageFilter<Eigen::Array3d>> filter_{nullptr};
};



}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
