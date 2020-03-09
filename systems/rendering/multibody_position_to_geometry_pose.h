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
 * The position input must be a vector whose length matches either the
 * number of positions in the MultibodyPlant or the number of states (based
 * on the optional argument in the constructor).  This option to pass the full
 * state vector is provided only for convenience -- only the position values
 * will affect the outputs.
 *
 * @tparam_double_only
 * @ingroup visualization
 */
template <typename T>
class MultibodyPositionToGeometryPose final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPositionToGeometryPose)

  /**
   * The %MultibodyPositionToGeometryPose holds an internal, non-owned
   * reference to the MultibodyPlant object so you must ensure that @p plant
   * has a longer lifetime than `this` %MultibodyPositionToGeometryPose system.
   *
   * @param input_multibody_state If true, the vector input port will be the
   * size of the `plant` *state* vector.  If false, it will be the size
   * of the `plant` *position* vector.  In both cases, only the
   * positions will affect the output.  @default false.
   *
   * @throws if `plant` is not finalized and registered with a SceneGraph.
   */
  explicit MultibodyPositionToGeometryPose(
      const multibody::MultibodyPlant<T>& plant,
      bool input_multibody_state = false);

  /**
   * The %MultibodyPositionToGeometryPose owns its internal plant.
   *
   * @param input_multibody_state If true, the vector input port will be the
   * size of the `plant` *state* vector.  If false, it will be the size
   * of the `plant` *position* vector.  In both cases, only the
   * positions will affect the output.  @default: false.
   *
   * @throws if `owned_plant` is not finalized and registered with a SceneGraph.
   */
  explicit MultibodyPositionToGeometryPose(
      std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
      bool input_multibody_state = false);

  ~MultibodyPositionToGeometryPose() override = default;

  const multibody::MultibodyPlant<T>& multibody_plant() const { return plant_; }

  /** Returns true if this system owns its MultibodyPlant. */
  bool owns_plant() const { return owned_plant_ != nullptr; }

  /** Returns the multibody position input port. */
  const InputPort<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  /** Returns the geometry::FramePoseVector output port. */
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

 private:
  // Configure the input/output ports and prepare for calculation.
  // @pre plant_ must reference a valid MBP.
  void Configure(bool input_multibody_state);

  void CalcGeometryPose(const Context<T>& context, AbstractValue* poses) const;

  // NOTE: The constructor's correctness depends on these two members declared
  // in this order (plant_ followed by owned_plant_). Do not change them.
  const multibody::MultibodyPlant<T>& plant_;
  // The optionally owned plant. If not null, owned_plant_ == &plant_ must be
  // true.
  const std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_;

  // This is a context of the plant_ system, which is only owned here to avoid
  // runtime allocation.  It contains no relevant state.
  mutable std::unique_ptr<Context<T>> plant_context_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
