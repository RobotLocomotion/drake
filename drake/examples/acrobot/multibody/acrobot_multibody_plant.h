#pragma once

#include <memory>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/modeler/multibody_modeler.h"

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
template <typename T>
class AcrobotMultibodyPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotMultibodyPlant)

  AcrobotMultibodyPlant(
      double m1 = 1.0,
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

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit AcrobotMultibodyPlant(const AcrobotMultibodyPlant<U>&);
  
  ///@{
  /// Manipulator equation of Acrobot: H * qdotdot + C = B*u.
  /// H[2x2] is the mass matrix.
  /// C[2x1] includes the Coriolis term, gravity term and the damping term, i.e.
  /// C[2x1] = Coriolis(q,v)*v + g(q) + [b1*theta1;b2*theta2]
  Vector2<T> VectorC(const systems::Context<T>& context) const;
  Matrix2<T> MatrixH(const systems::Context<T>& context) const;
  ///@}

  // getters for robot parameters
  double m1() const { return m1_; }
  double m2() const { return m2_; }
  double l1() const { return l1_; }
  double l2() const { return l2_; }
  double lc1() const { return lc1_; }
  double lc2() const { return lc2_; }
  double Ic1() const { return Ic1_; }
  double Ic2() const { return Ic2_; }
  double b1() const { return b1_; }
  double b2() const { return b2_; }
  double g() const { return g_; }

  const multibody::MultibodyModeler<T>& get_modeler() const {
    return modeler_;
  }

  const multibody::Link<T>& get_link1() const {
    return *link1_;
  }

  const multibody::Link<T>& get_link2() const {
    return *link2_;
  }

  const multibody::RevoluteJoint<T>& get_shoulder_joint() const {
    return *shoulder_;
  }

  const multibody::RevoluteJoint<T>& get_elbow_joint() const {
    return *elbow_;
  }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  // Override of context construction so that we can delegate it to
  // MultibodyModeler.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void OutputState(const systems::Context<T>& context,
                   systems::BasicVector<T>* state_port_value) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void BuildMultibodyModeler();

  // TODO(amcastro-tri): Declare these as parameters in the context.
  const double m1_, m2_, l1_, l2_, lc1_, lc2_, Ic1_, Ic2_, b1_, b2_, g_;

  // Quantities that occur often.
  const double I1_ = Ic1_ + m1_ * lc1_ * lc1_;
  const double I2_ = Ic2_ + m2_ * lc2_ * lc2_;
  const double m2l1lc2_ = m2_ * l1_ * lc2_;
  
  multibody::MultibodyModeler<T> modeler_;
  const multibody::Link<T>* link1_{nullptr};
  const multibody::Link<T>* link2_{nullptr};
  const multibody::RevoluteJoint<T>* shoulder_{nullptr};
  const multibody::RevoluteJoint<T>* elbow_{nullptr};
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake

// Disable scalar conversion from/to symbolic::Expresion.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<examples::acrobot::AcrobotMultibodyPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
