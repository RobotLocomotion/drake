#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/systems/controllers/model_based_controller_base.h"
#include "drake/systems/controllers/pid_controller.h"

namespace drake {
namespace systems {

/**
 * A state feedback controller that uses a PidController to generate desired
 * accelerations, which are then converted into torques using InverseDynamics.
 * More specifically, the output of this controller is:
 * `torque = inverse_dynamics(q, v, vd_d)`,
 * where `vd_d = kp(q* - q) + kd(v* - v) + ki int(q* - q) + vd*`.
 * `q` and `v` stand for the generalized position and velocity, and `vd` is
 * the generalized acceleration. `*` indicates reference values.
 *
 * This controller always has a BasicVector input port for estimated robot state
 * `(q, v)`, a BasicVector input port for reference robot state `(q*, v*)` and
 * a BasicVector output port for computed torque `torque`. A constructor flag
 * can be set to track reference acceleration `vd*` as well. When set, a
 * BasicVector input port is also declared, and it's content is used as `vd*`.
 * When unset, `vd*` is be treated as zero.
 *
 * Note that this class assumes the robot is fully actuated, its position
 * and velocity have the same dimension, it does not have a floating base.
 * If violated, the program will abort. It is discouraged to use this controller
 * for robots with closed kinematic loops.
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
   * @param kp Position gain
   * @param ki Integral gain
   * @param kd Velocity gain
   * @param has_reference_acceleration If true, there is an extra BasicVector
   * input port for `vd*`. If false, `vd*` is treated as zero, and no extra
   * input port is declared.
   */
  InverseDynamicsController(
      const std::string& model_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset,
      const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd,
      bool has_reference_acceleration);

  /**
   * Constructs the controller that clones a given RigidBodyTree.
   * @param robot Reference to the RigidBodyTree to be cloned.
   * @param kp Position gain
   * @param ki Integral gain
   * @param kd Velocity gain
   * @param has_reference_acceleration If true, there is an extra BasicVector
   * input port for `vd*`. If false, `vd*` is treated as zero, and no extra
   * input port is declared.
   */
  InverseDynamicsController(const RigidBodyTree<T>& robot, const VectorX<T>& kp,
                            const VectorX<T>& ki, const VectorX<T>& kd,
                            bool has_reference_acceleration);

  /**
   * Constructs the controller that takes ownership of a given RigidBodyTree
   * unique pointer.
   * @param robot Unique pointer whose ownership will be transfered to this
   * instance.
   * @param kp Position gain
   * @param ki Integral gain
   * @param kd Velocity gain
   * @param has_reference_acceleration If true, there is an extra BasicVector
   * input port for `vd*`. If false, `vd*` is treated as zero, and no extra
   * input port is declared.
   */
  InverseDynamicsController(std::unique_ptr<RigidBodyTree<T>> robot,
                            const VectorX<T>& kp, const VectorX<T>& ki,
                            const VectorX<T>& kd,
                            bool has_reference_acceleration);

  /**
   * Sets the integral part of the PidController to @p value.
   * @p value must be a column vector of the appropriate size.
   */
  void set_integral_value(Context<T>* context,
                          const Eigen::Ref<const VectorX<T>>& value) const;

  /**
   * Returns the input port for the reference acceleration.
   */
  const InputPortDescriptor<T>& get_input_port_desired_acceleration() const {
    DRAKE_DEMAND(has_reference_acceleration_);
    DRAKE_DEMAND(input_port_index_desired_acceleration_ >= 0);
    return Diagram<T>::get_input_port(input_port_index_desired_acceleration_);
  }

 private:
  void SetUp(const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd);

  PidController<T>* pid_{nullptr};
  const bool has_reference_acceleration_{false};
  int input_port_index_desired_acceleration_{-1};
};

}  // namespace systems
}  // namespace drake
