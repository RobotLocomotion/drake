#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace systems {
namespace sensors {

constexpr double kLcmStatusPeriod = 1.0 / 120.0;

/**
 * A structure used to store the attributes of a tracked rigid body. These are a
 * subset of the complete set of attributes which the real Optitrack hardware
 * maintains, but is typically sufficient for most applications.
 * @param id The rigid body ID as defined in the Optitrack software
 *        (e.g., Motive). This value must be unique and non-negative.
 * @param body_name The name of the Optitrack rigid body that is being tracked
 *        (not necessarily unique).
 * @param T_WF The pose of this rigid body w.r.t. the world.
 *
 */
struct TrackedBody {
  int id{};
  std::string body_name;
  Eigen::Isometry3d T_WF;
};

/**
 * Implements a class that allows a simulation to mock the basic output of the
 * Optitrack system. This class is useful when designing systems that interface
 * with actual Optitrack hardware, but are to be tested and vetted in
 * simulation. It maintains a set of attributes for each tracked object that is
 * a subset of the attributes maintained by the real hardware, but is typically
 * sufficient for most applications. Specifically each tracked rigid body is
 * described by its Optitrack ID, its name, and its pose w.r.t. the world. No
 * information about markers, marker sets, etc. is stored in this class. This
 * system has one abstract valued input port of type
 * systems::rendering::PoseBundle<double> and one abstract valued output port of
 * type std::vector<TrackedBody>. The output of this system can be connected to
 * an OptitrackFrameSender system which populates an optitrack_frame_t object
 * for publishing via LCM.
 * Optitrack bodies are created by specifying a set of frame names to track.
 * These frame names, provided at the time of construction, should have a match
 * within the set of names contained in the incoming PoseBundle object. Since
 * the incoming PoseBundle class requires frame names to be unique (when model
 * instance IDs are the same), there is also the option to provide a custom name
 * for each rigid body. These @p rigid_body_names need not be unique (as is the
 * case for the actual Optitrack hardware). This is because Optitrack IDs
 * (not names) are used to differentiate tracked rigid bodies. If no set of @p
 * rigid_body_names are provided, the corresponding frame name
 * from the incoming PoseBundle object is used.
 */
// TODO(rcory): Add support for markers and marker sets.
class OptitrackEncoder : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackEncoder)
  /**
   * Constructs an OptitrackEncoder object that defines a set of Optitrack rigid
   * bodies to track. Each rigid body is associated with a unique pose element
   * in the incoming PoseBundle object.
   * @param frame_name_to_id_map A mapping of frame names (contained in the
   *        incoming PoseBundle object) to Optitrack body ID. Each ID should be
   *        a unique non-negative integer.
   * @param rigid_body_names A vector of user defined names for the rigid
   *        bodies. These names do not necessarily have to match the incoming
   *        PoseBundle frame names, and they also not need be unique. The
   *        ordering of the names in this vector should correspond with the
   *        ordering of @p frame_name_to_id_map. If rigid_body_names.empty()
   *        returns true during construction, the associated frame names from
   *        @p frame_name_to_id_map are used in the same order as they appear.
   * @param optitrack_lcm_publish_period The update period for this system. This
   *        should match the publish period for the LCM publisher system.
   */
  OptitrackEncoder(
      const std::map<std::string, int>& frame_name_to_id_map,
      std::vector<std::string> rigid_body_names = std::vector<std::string>(),
      double optitrack_lcm_publish_period = kLcmStatusPeriod);

  /**
   * This InputPortDescriptor represents an abstract valued input port of type
   * PoseBundle<double>.
   */
  const systems::InputPortDescriptor<double>& get_pose_bundle_input_port()
  const {
    return get_input_port(pose_bundle_input_port_index_);
  }

  int get_pose_bundle_input_port_index() const {
    return pose_bundle_input_port_index_;
  }

  /**
   * This OutputPort represents an abstract valued output port of type
   * systems::sensors::TrackedBody.
   */
  const systems::OutputPort<double>& get_optitrack_output_port() const {
    return get_output_port(optitrack_output_port_index_);
  }

  int get_optitrack_output_port_index() const {
    return optitrack_output_port_index_;
  }

 private:
  // Checks whether the Optitrack ID is valid, i.e., is non-negative and unique.
  bool CheckIdValidity(const int id);

  std::vector<TrackedBody> MakeOutputValue() const;

  void CalcOutputValue(const systems::Context<double>& context,
                       std::vector<TrackedBody>* output) const;

  int pose_bundle_input_port_index_{-1};
  int optitrack_output_port_index_{-1};
  std::map<std::string, std::pair<std::string, int>> rigid_body_info_map_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
