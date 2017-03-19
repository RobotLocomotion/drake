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
/// the `quadrotor.urdf` model file.
template <typename T>
class QuadrotorPlant : public systems::LeafSystem<T> {
 public:
  QuadrotorPlant();
  QuadrotorPlant(double m_arg, double L_arg, const Matrix3<T>& I_arg,
                 double kF_arg, double kM_arg);

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  ~QuadrotorPlant() override;

  QuadrotorPlant<AutoDiffXd>* DoToAutoDiffXd() const override;

  int get_input_size() const { return kInputDimension; }

  int get_num_states() const { return kStateDimension; }

  void set_state(systems::Context<T>* context, const VectorX<T>& x) const {
    context->get_mutable_continuous_state_vector()->SetFromVector(x);
  }

  T  m() const { return m_; }
  T  g() const { return g_; }

 protected:
  void DoCalcOutput(const systems::Context<T> &context,
                    systems::SystemOutput<T> *output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T> &context,
      systems::ContinuousState<T> *derivatives) const override;

  // TODO(naveenoid): Declare these as parameters in the context.
 private:
  const double g_{9.81},  // Gravitational acceleration (m/s^2).
      m_{0.5},            // Mass of the robot (kg).
      L_{0.175},          // Length of the arms (m).
      kF_{1.0},           // Force input constant.
      kM_{0.0245};        // Moment input constant.
  int kStateDimension{12}, kInputDimension{4};
  const Matrix3<T> I_{
      ((Eigen::Matrix3d() << 0.0023, 0, 0, 0, 0.0023, 0, 0, 0, 0.0040)
           .finished())};  // Moment of Inertia about the Center of Mass
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
