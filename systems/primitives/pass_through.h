#pragma once

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A pass through system with input `u` and output `y = u`. This is
/// mathematically equivalent to a Gain system with its gain equal to one.
/// However this system incurs no computational cost.
/// The input to this system directly feeds through to its output.
/// This system is used, for instance, in PidController which is a Diagram
/// composed of simple framework primitives. In this case a PassThrough is used
/// to connect the exported input of the Diagram to the inputs of the Gain
/// systems for the proportioanal and integral constants of the controller. This
/// is necessary to provide an output port to which the internal Gain subsystems
/// connect. In this case the PassThrough is effectively creating an output port
/// that feeds through the input to the Diagram and that can now be connected to
/// the inputs of the inner subsystems to the Diagram.
/// A detailed discussion of the PidController can be found at
/// https://github.com/RobotLocomotion/drake/pull/3132.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// This class uses Drake's `-inl.h` pattern. When seeing linker errors from
/// this class, please refer to https://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// @ingroup primitive_systems
template <typename T>
class PassThrough final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PassThrough)

  /// Constructs a pass through system (`y = u`).
  /// @param vector_size number of elements in the signal to be processed.
  explicit PassThrough(int vector_size)
      : PassThrough(vector_size, nullptr) {}

  /// Constructs a pass through system (`y = u`).
  /// @param abstract_model_value A model abstract value.
  explicit PassThrough(const AbstractValue& abstract_model_value)
      : PassThrough(-1, abstract_model_value.Clone()) {}

  /// Scalar-type converting copy constructor.
  /// See @ref system_scalar_conversion.
  template <typename U>
  explicit PassThrough(const PassThrough<U>&);

  virtual ~PassThrough() = default;

  // TODO(eric.cousineau): Possibly share single port interface with
  // ZeroOrderHold (#6490).

  /// Returns the sole input port.
  const InputPort<T>& get_input_port() const {
    DRAKE_ASSERT(input_port_ != nullptr);
    return *input_port_;
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    DRAKE_ASSERT(output_port_ != nullptr);
    return *output_port_;
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 private:
  // Allow different specializations to access each other's private data.
  template <typename U> friend class PassThrough;

  // All of the other constructors delegate here.
  PassThrough(int vector_size,
              std::unique_ptr<const AbstractValue> abstract_model_value);

  /// Sets the output port to equal the input port.
  void DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const;

  // Same as `DoCalcVectorOutput`, but for abstract values.
  void DoCalcAbstractOutput(
      const Context<T>& context,
      AbstractValue* output) const;

  bool is_abstract() const { return abstract_model_value_ != nullptr; }

  const std::unique_ptr<const AbstractValue> abstract_model_value_;

  // We store our port pointers so that DoCalcVectorOutput's access to the
  // input_port_->Eval is inlined (without any port-count bounds checking).
  const InputPort<T>* input_port_{};
  const OutputPort<T>* output_port_{};
};

// TODO(amcastro-tri): remove the vector_size parameter from the constructor
// once #3109 supporting automatic sizes is resolved.
template <typename T>
PassThrough<T>::PassThrough(
    int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::PassThrough>()),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    BasicVector<T> model_value(vector_size);
    input_port_ = &this->DeclareVectorInputPort(model_value);
    output_port_ = &this->DeclareVectorOutputPort(
        model_value, &PassThrough::DoCalcVectorOutput);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    input_port_ = &this->DeclareAbstractInputPort(*abstract_model_value_);
    // Use the std::function<> overloads to work with `AbstractValue` type
    // directly and maintain type erasure.
    auto abstract_value_allocator = [this]() {
      return abstract_model_value_->Clone();
    };
    namespace sp = std::placeholders;
    output_port_ = &this->DeclareAbstractOutputPort(
        abstract_value_allocator,
        std::bind(&PassThrough::DoCalcAbstractOutput, this, sp::_1, sp::_2));
  }
}

template <typename T>
template <typename U>
PassThrough<T>::PassThrough(const PassThrough<U>& other)
    : PassThrough(other.is_abstract() ? -1 : other.get_input_port().size(),
                  other.is_abstract() ? other.abstract_model_value_->Clone()
                                      : nullptr) {}

template <typename T>
void PassThrough<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = get_input_port().Eval(context);
  DRAKE_ASSERT(input.size() == output->size());
  output->get_mutable_value() = input;
}

template <typename T>
void PassThrough<T>::DoCalcAbstractOutput(const Context<T>& context,
                                          AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  output->SetFrom(this->get_input_port().template Eval<AbstractValue>(context));
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PassThrough)
