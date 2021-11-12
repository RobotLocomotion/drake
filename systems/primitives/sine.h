#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A sine system which outputs `y = a * sin(f * t + p)` and first and second
/// derivatives w.r.t. the time parameter `t`. The block parameters are:
/// `a` the amplitude, `f` the frequency (radians/second), and `p` the phase
/// (radians), all of which are constant vectors provided at construction time.
/// This system has one or zero input ports and three vector valued output ports
/// (`y` and its first two derivatives). The user can specify whether to use
/// simulation time as the source of values for the time variable or an external
/// source. If an external time source is specified, the system is created with
/// an input port for the time source. Otherwise, the system is created with
/// zero input ports.
///
/// @system
/// name: Sine
/// input_ports:
/// - <span style="color:gray">u0</span>
/// output_ports:
/// - y0
/// - y1
/// - y2
/// @endsystem
///
/// Port `u0` is present if and only if the constructor parameter
/// `is_time_based` is false.
///
/// Port `y0` emits the value `y`; port `y1` emits the first derivative; port
/// `y2` emits the second derivative.
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class Sine final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Sine)

  /// Constructs a %Sine system where the amplitude, frequency, and phase is
  /// applied to every input.
  ///
  /// @param[in] amplitude the sine wave amplitude
  /// @param[in] frequency the sine wave frequency (radians/second)
  /// @param[in] phase the sine wave phase (radians)
  /// @param[in] size number of elements in the output signal.
  /// @param[in] is_time_based indicates whether to use the simulation time as
  ///            the source for the sine wave time variable, or use an external
  ///            source, in which case an input port of size @p size is created.
  Sine(double amplitude, double frequency, double phase, int size,
       bool is_time_based = true);

  /// Constructs a %Sine system where different amplitudes, frequencies, and
  /// phases can be applied to each sine wave.
  ///
  /// @param[in] amplitudes the sine wave amplitudes
  /// @param[in] frequencies the sine wave frequencies (radians/second)
  /// @param[in] phases the sine wave phases (radians)
  /// @param[in] is_time_based indicates whether to use the simulation time as
  ///            the source for the sine wave time variable, or use an external
  ///            source, in which case an input port is created.
  explicit Sine(const Eigen::VectorXd& amplitudes,
                const Eigen::VectorXd& frequencies,
                const Eigen::VectorXd& phases,
                bool is_time_based = true);

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit Sine(const Sine<U>&);

  /// Returns the amplitude constant. This method should only be called if the
  /// amplitude can be represented as a scalar value, i.e., every element in the
  /// amplitude vector is the same. It will abort if the amplitude cannot be
  /// represented as a single scalar value.
  double amplitude() const;

  /// Returns the frequency constant. This method should only be called if the
  /// frequency can be represented as a scalar value, i.e., every element in the
  /// frequency vector is the same. It will abort if the frequency cannot be
  /// represented as a single scalar value.
  double frequency() const;

  /// Returns the phase constant. This method should only be called if the phase
  /// can be represented as a scalar value, i.e., every element in the phase
  /// vector is the same. It will abort if the phase cannot be represented as a
  /// single scalar value.
  double phase() const;

  /// Returns a boolean indicting whether to use simulation time as the source
  /// of values for the time variable or an external source. Returns true if the
  /// simulation time is used as the source, and returns false otherwise.
  bool is_time_based() const;

  /// Returns the amplitude vector constant.
  const Eigen::VectorXd& amplitude_vector() const;

  /// Returns the frequency vector constant.
  const Eigen::VectorXd& frequency_vector() const;

  /// Returns the phase vector constant.
  const Eigen::VectorXd& phase_vector() const;

 private:
  void CalcValueOutput(const Context<T>& context,
                       BasicVector<T>* output) const;
  void CalcFirstDerivativeOutput(const Context<T>& context,
                                 BasicVector<T>* output) const;
  void CalcSecondDerivativeOutput(const Context<T>& context,
                                  BasicVector<T>* output) const;

  void CalcArg(const Context<T>& context, VectorX<T>* arg) const;


  const Eigen::VectorXd amplitude_;
  const Eigen::VectorXd frequency_;
  const Eigen::VectorXd phase_;
  const bool is_time_based_;
  bool is_const_amplitude_{false};
  bool is_const_frequency_{false};
  bool is_const_phase_{false};

  int value_output_port_index_{-1};
  int first_derivative_output_port_index_{-1};
  int second_derivative_output_port_index_{-1};
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Sine)
