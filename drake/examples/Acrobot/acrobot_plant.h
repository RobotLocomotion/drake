#pragma once

#pragma once

#include <memory>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace acrobot {

/// A model of a simple pendulum
/// @f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = u @f]
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class AcrobotPlant : public systems::LeafSystem<T> {
 public:
  AcrobotPlant();

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  // LeafSystem<T> override.
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;

  // LeafSystem<T> override.
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

  // System<T> override.
  AcrobotPlant<AutoDiffXd>* DoToAutoDiffXd() const override;

 private:
  // TODO(russt): Declare these as parameters in the context.
  const double m1{1.0},  // Mass of link 1 (kg).
      m2{1.0},           // Mass of link 2 (kg).
      l1{1.0},           // Length of link 1 (m).
      l2{2.0},           // Length of link 2 (m).
      lc1{0.5},  // Vertical distance from shoulder joint to center of mass of
                 // link 1 (m).
      lc2{1.0},  // Vertical distance from elbox joint to center of mass of link
                 // 2 (m).
      Ic1{.083},  // Inertia of link 1 about the center of mass of link 1
                  // (kg*m^2).
      Ic2{.33},   // Inertia of link 2 about the center of mass of link 2
                  // (kg*m^2).
      b1{0.1},    // Damping coefficient of the shoulder joint (kg*m^2/s).
      b2{0.1},    // Damping coefficient of the elbow joint (kg*m^2/s).
      g{9.81};    // Gravitational constant (m/s^2).
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
