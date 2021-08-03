#pragma once

#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "drake/common/constants.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/input_port_base.h"
#include "drake/systems/framework/value_to_abstract_value.h"

namespace drake {
namespace systems {

// Break the System <=> InputPort physical dependency cycle.  InputPorts are
// decorated with a back-pointer to their owning System<T>, but that pointer is
// forward-declared here and never dereferenced within this file.
template <typename T>
class System;

/** An InputPort is a System resource that describes the kind of input a
System accepts, on a given port. It does not directly contain any runtime
input port data; that is always contained in a Context. The actual value will
be either the value of an OutputPort to which this is connected, or a fixed
value set in a Context.

@tparam_default_scalar
*/
template <typename T>
class InputPort final : public InputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPort)

  /** Returns a reference to the up-to-date value of this input port contained
  in the given Context. This is the preferred way to obtain an input port's
  value since it will not be recalculated once up to date.

  If the value is not already up to date with respect to its prerequisites, it
  will recalculate an up-to-date value before the reference is returned. The
  recalculation may be arbitrarily expensive, but Eval() is constant time and
  _very_ fast if the value is already up to date.

  @tparam ValueType The type of the const-reference returned by this method.
  When omitted, the return type is an `Eigen::VectorBlock` (this is only valid
  when this is a vector-valued port).  For abstract ports, the `ValueType`
  either can be the declared type of the port (e.g., `lcmt_iiwa_status`), or
  else in advanced use cases can be `AbstractValue` to get the type-erased
  value.

  @return reference to the up-to-date value; if a ValueType is provided, the
  return type is `const ValueType&`; if a ValueType is omitted, the return type
  is `Eigen::VectorBlock<const VectorX<T>>`.

  @throw std::exception if the port is not connected.  (Use HasValue() to check
  first, if necessary.)

  @pre The input port is vector-valued (when no ValueType is provided).
  @pre The input port is of type ValueType (when ValueType is provided).
  */
#ifdef DRAKE_DOXYGEN_CXX
  template <typename ValueType = Eigen::VectorBlock<const VectorX<T>>>
  const ValueType& Eval(const Context<T>& context) const;
#else
  // Without a template -- return Eigen.
  Eigen::VectorBlock<const VectorX<T>> Eval(const Context<T>& context) const {
    return Eval<BasicVector<T>>(context).get_value();
  }
  // With ValueType == AbstractValue, we don't need to downcast.
  template <typename ValueType, typename = std::enable_if_t<
      std::is_same_v<AbstractValue, ValueType>>>
  const AbstractValue& Eval(const Context<T>& context) const {
    ValidateSystemId(context.get_system_id());
    return DoEvalRequired(context);
  }
  // With anything but a BasicVector subclass, we can just DoEval then cast.
  template <typename ValueType, typename = std::enable_if_t<
      !std::is_same_v<AbstractValue, ValueType> && (
        !std::is_base_of_v<BasicVector<T>, ValueType> ||
        std::is_same_v<BasicVector<T>, ValueType>)>>
  const ValueType& Eval(const Context<T>& context) const {
    ValidateSystemId(context.get_system_id());
    return PortEvalCast<ValueType>(DoEvalRequired(context));
  }
  // With a BasicVector subclass, we need to downcast twice.
  template <typename ValueType, typename = std::enable_if_t<
      std::is_base_of_v<BasicVector<T>, ValueType> &&
      !std::is_same_v<BasicVector<T>, ValueType>>>
  const ValueType& Eval(const Context<T>& context, int = 0) const {
    return PortEvalCast<ValueType>(Eval<BasicVector<T>>(context));
  }
#endif  // DRAKE_DOXYGEN_CXX

  /** Provides a fixed value for this %InputPort in the given Context. If the
  port is already connected, this value will override the connected source
  value. (By "connected" we mean that the port appeared in a
  DiagramBuilder::Connect() call.)

  For vector-valued input ports, you can provide an Eigen vector expression,
  a BasicVector object, or a scalar (treated as a Vector1). In each of these
  cases the value is copied into a `Value<BasicVector>`. If the original
  value was a BasicVector-derived object, its concrete type is maintained
  although the stored type is still `Value<BasicVector>`. The supplied vector
  must have the right size for the vector port or an std::logic_error is thrown.

  For abstract-valued input ports, you can provide any ValueType that is
  compatible with the model type provided when the port was declared. If the
  type has a copy constructor it will be copied into a `Value<ValueType>`
  object for storage. Otherwise it must have an accessible `Clone()` method and
  it is stored using the type returned by that method, which must be ValueType
  or a base class of ValueType. Eigen objects and expressions are not
  accepted directly, but you can store then in abstract ports by providing
  an already-abstract object like `Value<MatrixXd>(your_matrix)`.

  The returned FixedInputPortValue reference may be used to modify the input
  port's value subsequently using the appropriate FixedInputPortValue method,
  which will ensure that cache invalidation notifications are delivered.

  @tparam ValueType The type of the supplied `value` object. This will be
      inferred so no template argument need be specified. The type must be
      copy constructible or have an accessible `Clone()` method.

  @param[in,out] context A Context that is compatible with the System that
                         owns this port.
  @param[in]     value   The fixed value for this port. Must be convertible
                         to the input port's data type.
  @returns a reference to the FixedInputPortValue object in the Context that
           contains this port's value.

  @pre `context` is compatible with the System that owns this %InputPort.
  @pre `value` is compatible with this %InputPort's data type. */
  template <typename ValueType>
  FixedInputPortValue& FixValue(Context<T>* context,
                                const ValueType& value) const {
    DRAKE_DEMAND(context != nullptr);
    ValidateSystemId(context->get_system_id());
    const bool is_vector_port = (get_data_type() == kVectorValued);
    std::unique_ptr<AbstractValue> abstract_value =
        is_vector_port
            ? internal::ValueToVectorValue<T>::ToAbstract(__func__, value)
            : internal::ValueToAbstractValue::ToAbstract(__func__, value);
    return context->FixInputPort(get_index(), *abstract_value);
  }

  /** Returns true iff this port is connected or has had a fixed value provided
  in the given Context.  Beware that at the moment, this could be an expensive
  operation, because the value is brought up-to-date as part of this
  operation. */
  bool HasValue(const Context<T>& context) const {
    ValidateSystemId(context.get_system_id());
    return DoEvalOptional(context);
  }

  /** Returns a reference to the System that owns this input port. Note that
  for a Diagram input port this will be the Diagram, not the LeafSystem whose
  input port was exported. */
  const System<T>& get_system() const { return system_; }

  // A using-declaration adds these methods into our class's Doxygen.
  // (Placed in an order that makes sense for the class's table of contents.)
  using PortBase::get_name;
  using PortBase::GetFullDescription;
  using InputPortBase::get_index;
  using PortBase::get_data_type;
  using PortBase::size;
  using InputPortBase::is_random;
  using InputPortBase::get_random_type;
  using PortBase::ticket;

 private:
  friend class internal::FrameworkFactory;

  // See InputPortBase for the meaning of these parameters. The additional
  // `system` parameter must point to the same object as `system_interface`.
  // (They're separate because System is forward-declared so we can't cast.)
  InputPort(
      const System<T>* system,
      internal::SystemMessageInterface* system_interface,
      internal::SystemId system_id, std::string name,
      InputPortIndex index, DependencyTicket ticket, PortDataType data_type,
      int size, const std::optional<RandomDistribution>& random_type,
      EvalAbstractCallback eval)
      : InputPortBase(system_interface, system_id, std::move(name), index,
                      ticket, data_type, size, random_type, std::move(eval)),
        system_(*system) {
    DRAKE_DEMAND(system != nullptr);
    // Check the precondition on identical parameters; note that comparing as
    // void* is only valid because we have single inheritance.
    DRAKE_DEMAND(static_cast<const void*>(system) == system_interface);
  }

  const System<T>& system_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::InputPort)
