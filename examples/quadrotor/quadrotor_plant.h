#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace quadrotor {

/// The Quadrotor - an underactuated aerial vehicle. This version of the
/// Quadrotor is implemented to match the dynamics of the plant specified in
/// the `package://drake_models/skydio_2/quadrotor.urdf` model file.
///
/// @system
/// name: QuadrotorPlant
/// input_ports:
/// - propeller_force (optional)
/// output_ports:
/// - state
/// @endsystem
///
/// Note: If the propeller_force input port is not connected, then the force is
/// taken to be zero.
///
/// @tparam_default_scalar
template <typename T>
class QuadrotorPlant final : public systems::LeafSystem<T> {
 public:
  QuadrotorPlant();
  QuadrotorPlant(double m_arg, double L_arg, const Eigen::Matrix3d& I_arg,
                 double kF_arg, double kM_arg);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit QuadrotorPlant(const QuadrotorPlant<U>&);

  ~QuadrotorPlant() override;

  [[nodiscard]] double m() const { return m_; }
  [[nodiscard]] double g() const { return g_; }

  [[nodiscard]] double length() const { return L_; }
  [[nodiscard]] double force_constant() const { return kF_; }
  [[nodiscard]] double moment_constant() const { return kM_; }
  [[nodiscard]] const Eigen::Matrix3d& inertia() const { return I_; }

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // Allow different specializations to access each other's private data.
  template <typename>
  friend class QuadrotorPlant;

  // TODO(naveenoid): Declare these as parameters in the context.
  const double g_;           // Gravitational acceleration (m/s^2).
  const double m_;           // Mass of the robot (kg).
  const double L_;           // Length of the arms (m).
  const double kF_;          // Force input constant.
  const double kM_;          // Moment input constant.
  const Eigen::Matrix3d I_;  // Moment of Inertia about the Center of Mass
};

/// Generates an LQR controller to move to @p nominal_position. Internally
/// computes the nominal input corresponding to a hover at position @p x0.
/// @see systems::LinearQuadraticRegulator.
std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadrotorPlant<double>* quadrotor_plant,
    Eigen::Vector3d nominal_position);

}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::quadrotor::QuadrotorPlant);
