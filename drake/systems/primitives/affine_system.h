#pragma once

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A discrete OR continuous affine system.
///
/// If time_period>0.0, then the affine system will have the following discrete-
/// time state update:
///   @f[ x[n+1] = A x[n] + B u[n] + f_0, @f]
///
/// or if time_period==0.0, then the affine system will have the following
/// continuous-time state update:
///   @f[\dot{x} = A x + B u + f_0. @f]
///
/// In both cases, the system will have the output:
///   @f[y = C x + D u + y_0, @f]
/// where `u` denotes the input vector, `x` denotes the state vector, and
/// `y` denotes the output vector.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
/// @see LinearSystem
/// @see MatrixGain
template <typename T>
class AffineSystem : public LeafSystem<T> {
 public:
  /// Constructs an Affine system with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D` as well as fixed initial velocity offset `xDot0` and
  /// output offset `y0`.
  /// The coefficient matrices must obey the following dimensions :
  /// | Matrix  | Num Rows    | Num Columns |
  /// |:-------:|:-----------:|:-----------:|
  /// | A       | num states  | num states  |
  /// | B       | num states  | num inputs  |
  /// | C       | num outputs | num states  |
  /// | D       | num outputs | num inputs  |
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::VectorXd>& f0,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               const Eigen::Ref<const Eigen::VectorXd>& y0,
               double time_period = 0.0);

  /// The input to this system is direct feedthrough only if the coefficient
  /// matrix `D` is non-zero.
  bool has_any_direct_feedthrough() const override { return !D_.isZero(0.0); }

  /// Returns the input port containing the externally applied input.
  const SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the port containing the output state.
  const SystemPortDescriptor<T>& get_output_port() const;

  // Helper getter methods.
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::MatrixXd& B() const { return B_; }
  const Eigen::VectorXd& f0() const { return f0_; }
  const Eigen::MatrixXd& C() const { return C_; }
  const Eigen::MatrixXd& D() const { return D_; }
  const Eigen::VectorXd& y0() const { return y0_; }
  double time_period() const { return time_period_; }

 private:
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      drake::systems::DiscreteState<T>* updates) const override;

  // System<T> override.
  AffineSystem<AutoDiffXd>* DoToAutoDiffXd() const override;

  const Eigen::MatrixXd A_;
  const Eigen::MatrixXd B_;
  const Eigen::VectorXd f0_;
  const Eigen::MatrixXd C_;
  const Eigen::MatrixXd D_;
  const Eigen::VectorXd y0_;
  const int num_inputs_{0};
  const int num_outputs_{0};
  const int num_states_{0};
  const double time_period_{0.0};
};

}  // namespace systems
}  // namespace drake
