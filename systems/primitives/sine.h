#pragma once

#include <sstream>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_system.h"

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
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to https://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// To use other specific scalar types see sine-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
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

template <typename T>
Sine<T>::Sine(double amplitude, double frequency, double phase, int size,
              bool is_time_based)
    : Sine(Eigen::VectorXd::Ones(size) * amplitude,
           Eigen::VectorXd::Ones(size) * frequency,
           Eigen::VectorXd::Ones(size) * phase, is_time_based) {}

template <typename T>
Sine<T>::Sine(const Eigen::VectorXd& amplitudes,
              const Eigen::VectorXd& frequencies,
              const Eigen::VectorXd& phases,
              bool is_time_based)
      : LeafSystem<T>(SystemTypeTag<systems::Sine>{}),
      amplitude_(amplitudes), frequency_(frequencies), phase_(phases),
      is_time_based_(is_time_based) {
  // Ensure the incoming vectors are all the same size
  DRAKE_THROW_UNLESS(amplitudes.size() == frequencies.size());
  DRAKE_THROW_UNLESS(amplitudes.size() == phases.size());

  // Check each of the incoming vectors. For each vector, set a flag if every
  // element in that vector is the same.
  is_const_amplitude_ = amplitude_.isConstant(amplitude_[0]);
  is_const_frequency_ = frequency_.isConstant(frequency_[0]);
  is_const_phase_ = phase_.isConstant(phase_[0]);

  // If the Sine system is system time based, do not create an input port.
  // System time is used as the time variable in this case. If the Sine system
  // is not system time based, create an input port that contains the signal to
  // be used as the time variable.
  if (!is_time_based) {
    this->DeclareInputPort(kVectorValued, amplitudes.size());
  }
  value_output_port_index_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(amplitudes.size()),
          &Sine::CalcValueOutput).get_index();
  first_derivative_output_port_index_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(amplitudes.size()),
          &Sine::CalcFirstDerivativeOutput).get_index();
  second_derivative_output_port_index_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(amplitudes.size()),
          &Sine::CalcSecondDerivativeOutput).get_index();
}

template <typename T>
template <typename U>
Sine<T>::Sine(const Sine<U>& other)
    : Sine<T>(other.amplitude_vector(), other.frequency_vector(),
              other.phase_vector(), other.is_time_based()) {}

template <typename T>
double Sine<T>::amplitude() const {
  if (!is_const_amplitude_) {
    std::stringstream s;
    s << "The amplitude vector, [" << amplitude_ << "], cannot be represented "
      << "as a scalar value. Please use "
      << "drake::systems::Sine::amplitude_vector() instead.";
    throw std::logic_error(s.str());
  }
  return amplitude_[0];
}

template <typename T>
double Sine<T>::frequency() const {
  if (!is_const_frequency_) {
    std::stringstream s;
    s << "The frequency vector, [" << frequency_ << "], cannot be represented "
      << "as a scalar value. Please use "
      << "drake::systems::Sine::frequency_vector() instead.";
    throw std::logic_error(s.str());
  }
  return frequency_[0];
}

template <typename T>
double Sine<T>::phase() const {
  if (!is_const_phase_) {
    std::stringstream s;
    s << "The phase vector, [" << phase_ << "], cannot be represented as a "
      << "scalar value. Please use "
      << "drake::systems::Sine::phase_vector() instead.";
    throw std::logic_error(s.str().c_str());
  }
  return phase_[0];
}

template <typename T>
bool Sine<T>::is_time_based() const {
  return is_time_based_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::amplitude_vector() const {
  return amplitude_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::frequency_vector() const {
  return frequency_;
}

template <typename T>
const Eigen::VectorXd& Sine<T>::phase_vector() const {
  return phase_;
}

template <typename T>
void Sine<T>::CalcValueOutput(const Context<T>& context,
                              BasicVector<T>* output) const {
  VectorX<T> sine_arg;
  Sine::CalcArg(context, &sine_arg);

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block = amplitude_.array() * sine_arg.array().sin();
}

template <typename T>
void Sine<T>::CalcFirstDerivativeOutput(
    const Context<T>& context, BasicVector<T>* output) const {

  VectorX<T> cos_arg;
  Sine::CalcArg(context, &cos_arg);

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block =
      amplitude_.array() * frequency_.array() * cos_arg.array().cos();
}

template <typename T>
void Sine<T>::CalcSecondDerivativeOutput(
    const Context<T>& context, BasicVector<T>* output) const {

  VectorX<T> sine_arg;
  Sine::CalcArg(context, &sine_arg);

  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  output_block =
      - amplitude_.array() * frequency_.array().pow(2) * sine_arg.array().sin();
}

template <typename T>
void Sine<T>::CalcArg(
    const Context<T>& context, VectorX<T>* arg) const {

  if (is_time_based_) {
    VectorX<T> time_vec(amplitude_.size());
    time_vec.fill(context.get_time());
    *arg = frequency_.array() * time_vec.array() + phase_.array();
  } else {
    Eigen::VectorBlock<const VectorX<T>> input =
        this->get_input_port(0).Eval(context);
    *arg = frequency_.array() * input.array() + phase_.array();
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Sine)
