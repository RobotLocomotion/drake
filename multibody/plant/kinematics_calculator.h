#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/**
 * For a given MultibodyPlant, this system takes in a state vector input, and
 * outputs a corresponding context, whose state matches the input. This system
 * is motivated by a wide range of applications that need kinematics
 * computation but want to avoid expensive allocation of a Context on every
 * computation.
 *
 * @system{KinematicsCalculator,
 *   @input_port(state)
 *   @output_port(kinematics)
 * }
 */
template <typename T>
class KinematicsCalculator : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KinematicsCalculator)

  /**
   * Constructs a KinematicsCalculator.
   * @param[in] plant Const pointer to the MultibodyPlant. It is aliased
   * internally.
   */
  explicit KinematicsCalculator(const MultibodyPlant<T>* plant);

 private:
  const MultibodyPlant<T>& plant_;
};

}  // namespace multibody
}  // namespace drake
