#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/siso_vector_system.h"

namespace drake {
namespace systems {

/// An element-wise first order low pass filter system that filters the i-th
/// input uᵢ into the i-th output zᵢ. This system has one continuous state per
/// filtered input signal. Therefore, the i-th state of the system zᵢ evolves
/// according to: <pre>
///   żᵢ = -1/τᵢ (zᵢ - uᵢ)
/// </pre>
/// where τᵢ is the time constant of the i-th filter.
/// The i-th output of the system is given by: <pre>
///   yᵢ = zᵢ
/// </pre>
///
/// The transfer function for the i-th filter corresponds to: <pre>
///   H(s) = 1 / (1 + τᵢ s)
/// </pre>
///
/// The Bode plot for the i-th filter exhibits a cutoff frequency (angular
/// frequency) at 1/τᵢ and a gain of one. For frequencies higher than the cutoff
/// frequency, the Bode plot approaches a 20 dB per decade negative slope.
/// The Bode plot in phase exhibits a -90 degrees shift (lag) for frequencies
/// much larger than the cutoff frequency and a zero shift for low frequencies.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
template <typename T>
class FirstOrderLowPassFilter : public SisoVectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FirstOrderLowPassFilter)

  /// Constructs a %FirstOrderLowPassFilter system that filters all input
  /// signals with the same time constant, i.e. τᵢ = τ, ∀ i.
  ///
  /// @param[in] time_constant the time constant τ of the filter.
  /// @param[in] size number of elements in the signal to be processed.
  explicit FirstOrderLowPassFilter(double time_constant, int size = 1);

  /// Constructs a %FirstOrderLowPassFilter so that the i-th component of the
  /// input signal vector is low pass filtered with a time constant given in the
  /// i-th component τᵢ of the input `time_constants` vector.
  ///
  /// @param[in] time_constants Vector of time constants.
  explicit FirstOrderLowPassFilter(const VectorX<double>& time_constants);

  /// Returns the time constant of the filter for filters that have the same
  /// time constant τ for all signals.
  /// This method aborts if called on filters if with different time constants
  /// per input signal. @see get_time_constants_vector().
  double get_time_constant() const;

  /// Returns the vector of time constants for `this` filter.
  const VectorX<double>& get_time_constants_vector() const;

  /// Sets the initial conditions on the output value of the filtered signal.
  /// @param[in] context The current system context.
  /// @param[in] z0 The vector on initial conditions on the output value.
  void set_initial_output_value(Context<T>* context,
                                const Eigen::Ref<const VectorX<T>>& z0) const;

 protected:
  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const override;

  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override;

  // System<T> override. Returns a FirstOrderLowPassFilter<symbolic::Expression>
  // with the same time constants and dimensions as this Integrator.
  FirstOrderLowPassFilter<symbolic::Expression>* DoToSymbolic() const override;

  const VectorX<double> time_constants_;
};

}  // namespace systems
}  // namespace drake
