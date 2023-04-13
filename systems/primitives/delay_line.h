#pragma once

#include <memory>
#include <optional>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** A discrete delay line (also known as a "digital delay line" with an integer
number of samples).

@system
name: DelayLine
input_ports:
- u
output_ports:
- y
@endsystem

The initial output will be zero until num_samples have passed.

See @ref discrete_systems "Discrete Systems" for general information about
discrete systems in Drake, including how they interact with continuous
systems.

@note For an abstract-valued DelayLine where the model_value depents on the
      template parameter T, scalar-type conversion is not supported since
      AbstractValue does not support it.

@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class DelayLine final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DelayLine)

  /** Constructs a DelayLine with the given `period`, over a vector input of
  size `vector_size`. */
  DelayLine(int num_samples, double period, int vector_size);

  /** Constructs a DelayLine with the given `period`, over an abstract-valued
  input of type `model_value`. */
  DelayLine(int num_samples, double period, const AbstractValue& model_value);

  /** Scalar-type converting copy constructor.
  See @ref system_scalar_conversion. */
  template <typename U>
  explicit DelayLine(const DelayLine<U>& other);

  ~DelayLine() final;

  /** Reports the num_samples of this system. */
  int num_samples() const { return num_samples_; }

  /** Reports the period of this system (in seconds). */
  double period() const { return period_; }

 private:
  // Allow different specializations to access each other's private data.
  template <typename U>
  friend class DelayLine;

  DelayLine(int num_samples, double period, std::optional<int> vector_size,
            std::unique_ptr<const AbstractValue> model_value);

  // Samples the input into the queue of samples within the state.
  void Update(const Context<T>& context, State<T>* state) const;

  bool is_abstract() const { return model_value_ != nullptr; }

  const int num_samples_;
  const double period_;
  std::unique_ptr<const AbstractValue> model_value_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DelayLine)
