#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/model_based_controller_base.h"

namespace drake {
namespace systems {

// TODO(siyuan.feng): Lift the assumptions.

/**
 * A state feedback controller whose output is the sum of a PID controller
 * and a gravity compensator.
 *
 * Note that this class assumes the robot is fully actuated, its position
 * and velocity have the same dimension, and it does not have a floating base.
 */
template <typename T>
class InverseDynamicsController : public ModelBasedController<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamicsController)

  /**
   * Constructs the controller that instantiates a RigidBodyTree directly
   * from a model file. Assumes the model is connected to the world with a
   * multibody::joints::kFixed joint.
   * @param model_path Path to the model file.
   * @param world_offset X_WB, where B is the base frame of the model.
   */
  InverseDynamicsController(
      const std::string& model_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd,
      bool no_reference_acceleration);

  /**
   * Constructs the controller that clones a given RigidBodyTree.
   * @param robot Reference to the RigidBodyTree to be cloned.
   */
  InverseDynamicsController(const RigidBodyTree<T>& robot, const VectorX<T>& kp,
                            const VectorX<T>& ki, const VectorX<T>& kd,
                            bool no_reference_acceleration);

  /**
   * Constructs the controller that takes ownership of a given RigidBodyTree
   * unique pointer.
   * @param robot Unique pointer whose ownership will be transfered to this
   * instance.
   */
  InverseDynamicsController(std::unique_ptr<RigidBodyTree<T>> robot,
                            const VectorX<T>& kp, const VectorX<T>& ki,
                            const VectorX<T>& kd,
                            bool no_reference_acceleration);

  /**
   * Returns the input port for the desired state.
   */
  const InputPortDescriptor<T>& get_desired_acceleration_input_port() const {
    DRAKE_DEMAND(!no_reference_acceleration_);
    return Diagram<T>::get_input_port(2);
  }

 private:
  void SetUp(const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

  const bool no_reference_acceleration_{false};
};

}  // namespace systems
}  // namespace drake
