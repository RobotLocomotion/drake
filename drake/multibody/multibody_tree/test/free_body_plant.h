#pragma once

#include <memory>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/rpy_mobilizer.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @param m1 Mass of link 1 (kg).
/// @param m2 Mass of link 2 (kg).
/// @param l1 Length of link 1 (m).
/// @param l2 Length of link 2 (m).
/// @param lc1 Vertical distance from shoulder joint to center of mass of
/// link 1 (m).
/// @param lc2 Vertical distance from elbow joint to center of mass of
/// link 2 (m).
/// @param Ic1 Inertia of link 1 about the center of mass of link 1
/// (kg*m^2).
/// @param Ic2 Inertia of link 2 about the center of mass of link 2
/// (kg*m^2).
/// @param b1 Damping coefficient of the shoulder joint (kg*m^2/s).
/// @param b2 Damping coefficient of the elbow joint (kg*m^2/s).
/// @param g Gravitational constant (m/s^2).
///
/// The parameters are defaulted to values in Spong's paper (see
/// acrobot_spong_controller.cc for more details). Alternatively, an instance
/// of AcrobotMultibodyPlant using parameters of MIT lab's acrobot can be created by
/// calling the static method CreateAcrobotMIT();
///
/// Note that the Spong controller behaves differently on these two sets of
/// parameters. The controller works well on the first set of parameters,
/// which Spong used in his paper. In contrast, it is difficult to find a
/// functional set of gains to stabilize the robot about its upright fixed
/// point using the second set of parameters, which represent a real robot.
/// This difference illustrates limitations of the Spong controller.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template<typename T>
class FreeBodyPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreeBodyPlant)

  FreeBodyPlant(double I, double J);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit FreeBodyPlant(const FreeBodyPlant<U>&);

  double get_mass() const { return 1.0; }

  void set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_WB) const;

  Isometry3<T> CalcPoseInWorldFrame(
      const systems::Context<T>& context) const;

  SpatialVelocity<T> CalcSpatialVelocityInWorldFrame(
      const systems::Context<T>& context) const;

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  // Override of context construction so that we can delegate it to
  // MultibodyTree.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void OutputState(const systems::Context<T> &context,
                   systems::BasicVector<T> *state_port_value) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T> &context,
      systems::ContinuousState<T> *derivatives) const override;

  void BuildMultibodyTreeModel();

  double I_{0};
  double J_{0};
  MultibodyTree<T> model_;
  const RigidBody<T>* body_{nullptr};
  const RollPitchYawMobilizer<T>* mobilizer_{nullptr};
};

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
