#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
 * Base class for a discrete- or continuous-time, time-varying affine
 * system, with potentially time-varying coefficients.
 *
 * @system
 * name: TimeVaryingAffineSystem
 * input_ports:
 * - u0
 * output_ports:
 * - y0
 * @endsystem
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
 * @tparam_default_scalar
 * @ingroup primitive_systems
 * *
 * @see AffineSystem
 */
template <typename T>
class TimeVaryingAffineSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingAffineSystem)

  /// Returns the input port containing the externally applied input.
  const InputPort<T>& get_input_port() const;

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

  /// Configures the value that will be assigned to the state vector in
  /// `SetDefaultContext`. `x0` must be a vector of length `num_states`.
  void configure_default_state(const Eigen::Ref<const VectorX<T>>& x0);

  /// Configures the Gaussian distribution over state vectors used in the
  /// `SetRandomContext` methods.  The mean of the distribution will be the
  /// default state (@see configure_default_state()). `covariance` must have
  /// size `num_states` by `num_states` and must be symmetric and positive
  /// semi-definite.
  void configure_random_state(
      const Eigen::Ref<const Eigen::MatrixXd>& covariance);

  /// Returns the configured default state.  @see configure_default_state().
  const VectorX<T>& get_default_state() const { return x0_; }

  /// Returns the configured random state covariance.
  const Eigen::MatrixXd get_random_state_covariance() const {
    return Sqrt_Sigma_x0_ * Sqrt_Sigma_x0_;
  }

  double time_period() const { return time_period_; }
  int num_states() const { return num_states_; }
  int num_inputs() const { return num_inputs_; }
  int num_outputs() const { return num_outputs_; }

 protected:
  /// Constructor.
  ///
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  /// @param num_states size of the system's state vector
  /// @param num_inputs size of the system's input vector
  /// @param num_outputs size of the system's output vector
  /// @param time_period discrete update period, or 0.0 to use continuous time
  TimeVaryingAffineSystem(SystemScalarConverter converter,
                          int num_states, int num_inputs, int num_outputs,
                          double time_period);

  /// Helper method.  Derived classes should call this from the
  /// scalar-converting copy constructor.
  template <typename U>
  void ConfigureDefaultAndRandomStateFrom(
      const TimeVaryingAffineSystem<U>& other);

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
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const override;

  /// Sets the initial conditions.
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override;

  /// Sets the random initial conditions.
  void SetRandomState(const Context<T>& context, State<T>* state,
      RandomGenerator* generator) const override;

 private:
  // For use by DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS... because the
  // function it needs to instantiate is protected.
  template<typename, typename>
  friend constexpr auto Make_Function_Pointers();

  const int num_states_{0};
  const int num_inputs_{0};
  const int num_outputs_{0};
  const double time_period_{0.0};

  VectorX<T> x0_;     // Default state.
  Eigen::MatrixXd Sqrt_Sigma_x0_;  // Square root of state covariance matrix.
};

/// A discrete OR continuous affine system (with constant coefficients).
///
/// @system
/// name: AffineSystem
/// input_ports:
/// - u0
/// output_ports:
/// - y0
/// @endsystem
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
/// @tparam_default_scalar
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
  ///
  /// Subclasses must use the protected constructor, not this one.
  AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::VectorXd>& f0,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               const Eigen::Ref<const Eigen::VectorXd>& y0,
               double time_period = 0.0);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit AffineSystem(const AffineSystem<U>&);

  /// Creates a unique pointer to AffineSystem<T> by decomposing @p dynamics and
  /// @p outputs using @p state_vars and @p input_vars.
  ///
  /// @throws std::exception if either @p dynamics or @p outputs is not
  /// affine in @p state_vars and @p input_vars.
  static std::unique_ptr<AffineSystem<T>> MakeAffineSystem(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& output,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& state_vars,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& input_vars,
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

 protected:
  /// Constructor that specifies scalar-type conversion support.
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  AffineSystem(SystemScalarConverter converter,
               const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::MatrixXd>& B,
               const Eigen::Ref<const Eigen::VectorXd>& f0,
               const Eigen::Ref<const Eigen::MatrixXd>& C,
               const Eigen::Ref<const Eigen::MatrixXd>& D,
               const Eigen::Ref<const Eigen::VectorXd>& y0,
               double time_period);

 private:
  void CalcOutputY(const Context<T>& context,
                   BasicVector<T>* output_vector) const final;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final;

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const final;

  const Eigen::MatrixXd A_;
  const Eigen::MatrixXd B_;
  const Eigen::VectorXd f0_;
  const Eigen::MatrixXd C_;
  const Eigen::MatrixXd D_;
  const Eigen::VectorXd y0_;
  const bool has_meaningful_C_{};
  const bool has_meaningful_D_{};
};

}  // namespace systems
}  // namespace drake
