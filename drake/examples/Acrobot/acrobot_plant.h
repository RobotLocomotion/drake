#pragma once

#include <memory>

#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace acrobot {


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
/// of AcrobotPlant using parameters of MIT lab's acrobot can be created by
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
template <typename T>
class AcrobotPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotPlant)

  AcrobotPlant(double m1 = 1.0,
               double m2 = 1.0,
               double l1 = 1.0,
               double l2 = 2.0,
               double lc1 = 0.5,
               double lc2 = 1.0,
               double Ic1 = .083,
               double Ic2 = .33,
               double b1 = 0.1,
               double b2 = 0.1,
               double g = 9.81);

  /// Creates an instance of AcrobotPlant using parameters of MIT lab's acrobot.
  static std::unique_ptr<AcrobotPlant<T>> CreateAcrobotMIT();

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  ///@{
  /// Manipulator equation of Acrobot: H * qdotdot + C = B*u.
  /// H[2x2] is the mass matrix.
  /// C[2x1] includes the Coriolis term, gravity term and the damping term, i.e.
  /// C[2x1] = Coriolis(q,v)*v + g(q) + [b1*theta1;b2*theta2]
  Vector2<T> VectorC(const AcrobotStateVector<T>& x) const;
  Matrix2<T> MatrixH(const AcrobotStateVector<T>& x) const;
  ///@}

  // getters for robot parameters
  T m1() const { return m1_; }
  T m2() const { return m2_; }
  T l1() const { return l1_; }
  T l2() const { return l2_; }
  T lc1() const { return lc1_; }
  T lc2() const { return lc2_; }
  T Ic1() const { return Ic1_; }
  T Ic2() const { return Ic2_; }
  T b1() const { return b1_; }
  T b2() const { return b2_; }
  T g() const { return g_; }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // System<T> override.
  AcrobotPlant<AutoDiffXd>* DoToAutoDiffXd() const override;

  // TODO(russt): Declare these as parameters in the context.

  const double m1_, m2_, l1_, l2_, lc1_, lc2_, Ic1_, Ic2_, b1_, b2_, g_;
  const double I1_ = Ic1_ + m1_ * lc1_ * lc1_;
  const double I2_ = Ic2_ + m2_ * lc2_ * lc2_;
  const double m2l1lc2_ = m2_ * l1_ * lc2_;  // Quantities that occur often.
};

/// Constructs the Acrobot with (only) encoder outputs.
template <typename T>
class AcrobotWEncoder : public systems::Diagram<T> {
 public:
  explicit AcrobotWEncoder(bool acrobot_state_as_second_output = false);

  const AcrobotPlant<T>* acrobot_plant() const { return acrobot_plant_; }

  AcrobotStateVector<T>* get_mutable_acrobot_state(
      systems::Context<T>* context) const;

 private:
  AcrobotPlant<T>* acrobot_plant_{nullptr};
};

/// Constructs the LQR controller for stabilizing the upright fixed point using
/// default LQR cost matrices which have been tested for this system.
std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>& acrobot);

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
