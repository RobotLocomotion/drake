#pragma once

#include <memory>
#include <optional>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** An input-output delay line.

This system can model either discrete delay (also known as a "digital delay
line" with an integer number of samples), or continuous delay (with no bandwith
limit, i.e., captures every per-step input).

@system
name: DelayLine
input_ports:
- u
output_ports:
- y
@endsystem

XXX discrete
Until num_samples have passed, the initial output will be zero (for a vector
port) or the model_value (for an abstract port).

XXX continuous

See @ref discrete_systems "Discrete Systems" for general information about
discrete systems in Drake, including how they interact with continuous
systems.

@note For an abstract-valued DelayLine where the model_value depends on the
 template parameter T, scalar-type conversion is not supported since
 AbstractValue does not support it.

@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class DelayLine final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DelayLine)

  /** Constructs a discrete delay line (also known as a "digital delay line")
  with an integer number of samples, for vector input of size `vector_size`.
  @param vector_size The size of the input port and output port.
  @param num_samples The number of samples to buffer as State; must be strictly
   positive.
  @param period The first sample will occur at t = `offset`, and it will recur
   at every `period` thereafter.
  @param offset The first sample will occur at t = `offset`, and it will recur
   at every `period` thereafter. */
  DelayLine(int vector_size, int num_samples, double period,
            double offset = 0.0);

  /** Constructs a discrete delay line (also known as a "digital delay line")
  with an integer number of samples, for abstract-valued input of the same type
  as `model_value`.
  @param model_value The default output value prior to filling the State buffer.
   Also serves to specify the specific type of Value expect as input.
  @param num_samples The number of samples to buffer as State; must be strictly
   positive.
  @param period The first sample will occur at t = `offset`, and it will recur
   at every `period` thereafter.
  @param offset The first sample will occur at t = `offset`, and it will recur
   at every `period` thereafter. */
  DelayLine(const AbstractValue& model_value, int num_samples, double period,
            double offset = 0.0);

  /** Constructs a continuous delay line for vector input of size `vector_size`.
  @param vector_size The size of the input port and output port.
  @param delay The total delay, in seconds; must be strictly positive.
  @note Continuous delay supports unlimited bandwidth by using per-step events
   to sample every single input value into state; be aware of the memory
   requirement this implies. */
  DelayLine(int vector_size, double delay);

  /** Constructs a continuous delay line for abstract-valued input of the same
  type as `model_value`.
  @param model_value The default output value prior to reaching the `delay`
   time.  Also serves to specify the specific type of Value expect as input.
  @param delay The total delay, in seconds; must be strictly positive.
  @note Continuous delay supports unlimited bandwidth by using per-step events
   to sample every single input value into state; be aware of the memory
   requirement this implies. */
  DelayLine(const AbstractValue& model_value, double delay);

  /** Scalar-type converting copy constructor.
  See @ref system_scalar_conversion. */
  template <typename U>
  explicit DelayLine(const DelayLine<U>& other);

  ~DelayLine() final;

  /** Returns the num_samples of this system, when using discrete delay.
  When using continuous delay, returns nullopt. */
  std::optional<int> num_samples() const { return num_samples_; }

  /** Returns the update period of this system (in seconds), when using discrete
  delay. When using continuous delay, returns nullopt. */
  std::optional<double> period() const { return period_; }

  /** Returns the initial update time of this system (in seconds), when using
  discrete delay. When using continuous delay, returns nullopt. */
  std::optional<double> offset() const { return offset_; }

  /** Returns the delay of this system (in seconds). When using discrete delay,
  this is `num_samples * period`. */
  double delay() const { ...; }

 private:
  // Allow different specializations to access each other's private data.
  template <typename U>
  friend class DelayLine;

  // All constructors delegate to this common constructor.
  DelayLine(std::optional<int> vector_size,
            std::unique_ptr<const AbstractValue> model_value, int num_samples,
            double period, double offset);

  // Samples the input into the queue of samples within the state.
  void Update(const Context<T>& context, State<T>* state) const;

  bool is_abstract() const { return model_value_ != nullptr; }

  const int num_samples_;
  const double period_;
  const double offset_;
  std::unique_ptr<const AbstractValue> model_value_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DelayLine)
