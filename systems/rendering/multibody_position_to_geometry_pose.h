#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace rendering {

/**
 * A direct-feedthrough system that converts a vector of joint positions
 * directly to a geometry::FramePoseVector<T> to behave like a
 * MultibodyPlant::get_geometry_pose_output_port().
 *
 * @system{ MultibodyPositionToGeometryPose,
 *          @input_port{position},
 *          @output_port{geometry_pose}
 * }
 *
 * The position input must be a vector whose length matches the number of
 * positions in the MultibodyPlant.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following kinds of T's are provided:
 *
 * - double
 *
 * @ingroup visualization
 * }
 */
template <typename T>
class MultibodyPositionToGeometryPose final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPositionToGeometryPose)

  /***
   * The %MultibodyPositionToGeometryPose holds an internal, non-owned
   * reference to the MultibodyPlant object so you must ensure that @p plant
   * has a longer lifetime than `this` %MultibodyPositionToGeometryPose system.
   *
   * @pre @p plant must be registered with a SceneGraph.
   * @pre @p plant must be finalized.
   */
  explicit MultibodyPositionToGeometryPose(
      const multibody::MultibodyPlant<T>& plant);

  ~MultibodyPositionToGeometryPose() override = default;

  /** Returns the multibody position input port. */
  const InputPort<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  /** Returns the geometry::FramePoseVector output port. */
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

 private:
  optional<bool> DoHasDirectFeedthrough(int, int) const final { return true; }

  void CalcGeometryPose(const Context<T>& context, AbstractValue* poses) const;

  const multibody::MultibodyPlant<T>& plant_;

  // This is a context of the plant_ system, which is only owned here to avoid
  // runtime allocation.  It contains no relevant state.
  mutable std::unique_ptr<Context<T>> plant_context_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
