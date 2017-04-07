#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
 * Base class for a discrete- or continuous-time, time-varying affine
 * system, with potentially time-varying coefficients.
 *
 * If `time_period > 0.0`, then the affine system will have the state update:
 *   @f[ x(t+h) = A(t) x(t) + B(t) u(t) + f_0(t), @f]
 * where `h` is the time_period.  If `time_period == 0.0`, then the system will
 * have the time derivatives:
 *   @f[ \dot{x}(t) = A(t) x(t) + B(t) u(t) + f_0(t), @f]
 * where `u` denotes the input vector, `x` denotes the state vector.
 *
 * In both cases, the system will have the output:
 *   @f[ y(t) = C(t) x(t) + D(t) u(t) + y_0(t), @f]
 * where `y` denotes the output vector.
 *
 * @tparam T The scalar element type, which must be a valid Eigen scalar.
 */
template <typename T>
class TimeVaryingAffineSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingAffineSystem)

  /// Returns the input port containing the externally applied input.
  const InputPortDescriptor<T>& get_input_port() const;

  /// Returns the output port containing the output state.
  const OutputPort<T>& get_output_port() const;

  /// @name Methods To Be Implemented by Subclasses
  ///
  /// Implementations must define these, and the returned matrices must
  /// be sized to match the `num_states`, `num_inputs`, and `num_outputs`
  /// specified in the constructor.
  /// @{
  virtual MatrixX<T> A(const T& t) const = 0;
  virtual MatrixX<T> B(const T& t) const = 0;
  virtual VectorX<T> f0(const T& t) const = 0;
  virtual MatrixX<T> C(const T& t) const = 0;
  virtual MatrixX<T> D(const T& t) const = 0;
  virtual VectorX<T> y0(const T& t) const = 0;
  /// @}

  double time_period() const { return time_period_; }
  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }
  int num_outputs() const { return num_outputs_; }

 protected:
  TimeVaryingAffineSystem(int num_states, int num_inputs, int num_outputs,
                          double time_period = 0.0);

  /// Computes @f[ y(t) = C(t) x(t) + D(t) u(t) + y_0(t), @f] with by calling
  /// `C(t)`, `D(t)`, and `y0(t)` with runtime size checks.  Derived classes
  /// may override this for performance reasons.
  virtual void CalcOutputY(const Context<T>& context,
                           BasicVector<T>* output_vector) const;

  /// Computes @f[ \dot{x}(t) = A(t) x(t) + B(t) u(t) + f_0(t), @f] with by
  /// calling `A(t)`, `B(t)`, and `f0(t)` with runtime size checks.  Derived
  /// classes may override this for performance reasons.
  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  /// Computes @f[ x(t+h) = A(t) x(t) + B(t) u(t) + f_0(t), @f] with by calling
  /// `A(t)`, `B(t)`, and `f0(t)` with runtime size checks.  Derived classes
  /// may override this for performance reasons.
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      drake::systems::DiscreteValues<T>* updates) const override;

 private:
  const int num_states_{0};
  const int num_inputs_{0};
  const int num_outputs_{0};
  const double time_period_{0.0};
};

/// A discrete OR continuous affine system (with constant coefficients).
///
/// Let `u` denote the input vector, `x` denote the state vector, and
/// `y` denote the output vector.
///
/// If `time_period > 0.0`, the affine system will have the following
/// discrete-time state update:
///   @f[ x(t+h) = A x(t) + B u(t) + f_0, @f]
/// where `h` is the time_period.
/// If `time_period == 0.0`, the affine system will have the following
/// continuous-time state update:
///   @f[\dot{x} = A x + B u + f_0. @f]
///
/// In both cases, the system will have the output:
///   @f[y = C x + D u + y_0, @f]
///
/// @tparam T The scalar element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
/// @see LinearSystem
/// @see MatrixGain
template <typename T>
class AffineSystem : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AffineSystem)

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

  /// @name Helper getter methods.
  /// @{
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::MatrixXd& B() const { return B_; }
  const Eigen::VectorXd& f0() const { return f0_; }
  const Eigen::MatrixXd& C() const { return C_; }
  const Eigen::MatrixXd& D() const { return D_; }
  const Eigen::VectorXd& y0() const { return y0_; }
  /// @}

  /// @name Implementations of TimeVaryingAffineSystem<T>'s pure virtual
  /// methods.
  /// @{
  MatrixX<T> A(const T&) const final { return MatrixX<T>(A_); }
  MatrixX<T> B(const T&) const final { return MatrixX<T>(B_); }
  VectorX<T> f0(const T&) const final { return VectorX<T>(f0_); }
  MatrixX<T> C(const T&) const final { return MatrixX<T>(C_); }
  MatrixX<T> D(const T&) const final { return MatrixX<T>(D_); }
  VectorX<T> y0(const T&) const final { return VectorX<T>(y0_); }
  /// @}

 private:
  void CalcOutputY(const Context<T>& context,
                   BasicVector<T>* output_vector) const final;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final;

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      drake::systems::DiscreteValues<T>* updates) const final;

  // System<T> override.
  AffineSystem<AutoDiffXd>* DoToAutoDiffXd() const final;
  AffineSystem<symbolic::Expression>* DoToSymbolic() const final;

  const Eigen::MatrixXd A_;
  const Eigen::MatrixXd B_;
  const Eigen::VectorXd f0_;
  const Eigen::MatrixXd C_;
  const Eigen::MatrixXd D_;
  const Eigen::VectorXd y0_;
};

}  // namespace systems
}  // namespace drake
