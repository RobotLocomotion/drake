#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "drake/common/constants.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/input_port_base.h"

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

@tparam T The mathematical type of the context, which must be a valid Eigen
          scalar.
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
      std::is_same<AbstractValue, ValueType>::value>>
  const AbstractValue& Eval(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(get_system_base().ThrowIfContextNotCompatible(context));
    return DoEvalRequired(context);
  }
  // With anything but a BasicVector subclass, we can just DoEval then cast.
  template <typename ValueType, typename = std::enable_if_t<
      !std::is_same<AbstractValue, ValueType>::value && (
        !std::is_base_of<BasicVector<T>, ValueType>::value ||
        std::is_same<BasicVector<T>, ValueType>::value)>>
  const ValueType& Eval(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(get_system_base().ThrowIfContextNotCompatible(context));
    return PortEvalCast<ValueType>(DoEvalRequired(context));
  }
  // With a BasicVector subclass, we need to downcast twice.
  template <typename ValueType, typename = std::enable_if_t<
      std::is_base_of<BasicVector<T>, ValueType>::value &&
      !std::is_same<BasicVector<T>, ValueType>::value>>
  const ValueType& Eval(const Context<T>& context, int = 0) const {
    return PortEvalCast<ValueType>(Eval<BasicVector<T>>(context));
  }
#endif  // DRAKE_DOXYGEN_CXX

#ifdef DRAKE_DOXYGEN_CXX
  /** Provides a fixed value for this %InputPort in the given Context. If the
  port is already connected, this value will override the connected source
  value. (By "connected" we mean that the port appeared in a
  DiagramBuilder::Connect() call.)

  If any Eigen vector object is provided, it is copied to a BasicVector object
  and this must be a vector port. A BasicVector-derived object may also be
  supplied and its concrete type will be preserved. Any other ValueType will
  be converted to an AbstractValue whose underlying type is `Value<ValueType>`
  and this must be an abstract port.

  The returned FixedInputPortValue reference may be used to modify the input
  port's value subsequently using the appropriate FixedInputPortValue method,
  which will ensure that cache invalidation notifications are delivered.

  @tparam ValueType The type of the supplied `value` object. This will be
      inferred so no template argument need be specified. The type must be
      copy constructible or have a public `Clone()` method.

  @param[in,out] context A Context that is compatible with the System that
                         owns this port.
  @param[in]     value   The fixed value for this port. Must be convertible
                         to the input port's data type.
  @returns a reference to the the FixedInputPortValue object in the Context
           that contains this port's value.

  @pre `context` must be compatible with the System that owns this %InputPort.
  @pre `value` must be compatible with this %InputPort's data type. */
  template <typename ValueType>
  FixedInputPortValue& FixValue(Context<T>* context,
                                const ValueType& value) const;
#else
  // The FixValue() methods here all use the ContextBase FixInputPort()
  // method that takes a unique_ptr<AbstractValue>, rather than the many
  // variants in Context<T>, in case we can deprecate those.

  // This signature is used for any Eigen vector type, but the argument is
  // copied to a BasicVector for the input port value.
  FixedInputPortValue& FixValue(
      Context<T>* context, const Eigen::Ref<const VectorX<T>>& vector) const {
    return context->FixInputPort(
        get_index(), std::make_unique<Value<BasicVector<T>>>(vector));
  }

  // BasicVector must be special-cased because we permit use of a concrete
  // BasicVector subtype that does not have its own Clone() method. That does
  // not meet the requirements of drake::is_cloneable.
  FixedInputPortValue& FixValue(Context<T>* context,
                                const BasicVector<T>& vector) const {
    return context->FixInputPort(
        get_index(), std::make_unique<Value<BasicVector<T>>>(vector.Clone()));
  }

  // This signature is used for AbstractValue or Value<U> arguments.
  FixedInputPortValue& FixValue(Context<T>* context,
                                const AbstractValue& abstract_value) const {
    return context->FixInputPort(get_index(), abstract_value.Clone());
  }

  // Special case char* to std::string to avoid ugly compilation messages
  // for this case, where the user's intent is obvious.
  FixedInputPortValue& FixValue(Context<T>* context,
                                const char* c_string) const {
    return FixValue(&*context, std::string(c_string));
  }

  // Returns true if ValueType is compatible with Eigen::Ref.
  template <typename ValueType>
  static constexpr bool is_eigen_refable() {
    return is_eigen_refable_helper<ValueType>(1);  // Any int will do here.
  }

  // This method won't instantiate if any of the non-templatized signatures
  // above can be used (after possible conversions). If we allowed this to
  // instantiate it would be chosen instead of performing those conversions.
  template <typename ValueType,
            typename = std::enable_if_t<
                !(is_eigen_refable<ValueType>() ||
                  std::is_base_of<AbstractValue, ValueType>::value ||
                  std::is_base_of<BasicVector<T>, ValueType>::value)>>
  FixedInputPortValue& FixValue(Context<T>* context,
                                const ValueType& value) const {
    static_assert(
        std::is_copy_constructible<ValueType>::value ||
            drake::is_cloneable<ValueType>::value,
        "FixValue(): an input port value type must be copy constructible or "
        "have a public Clone() method.");
    return FixValueHelper(&*context, value, 1, 1);
  }
#endif

  /** Returns true iff this port is connected or has had a fixed value provided
  in the given Context.  Beware that at the moment, this could be an expensive
  operation, because the value is brought up-to-date as part of this
  operation. */
  bool HasValue(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(get_system_base().ThrowIfContextNotCompatible(context));
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
  // `system` parameter must point to the same object as `system_base`.
  // (They're separate because System is forward-declared so we can't cast.)
  InputPort(
      const System<T>* system, internal::SystemMessageInterface* system_base,
      std::string name, InputPortIndex index, DependencyTicket ticket,
      PortDataType data_type, int size,
      const optional<RandomDistribution>& random_type,
      EvalAbstractCallback eval)
      : InputPortBase(system_base, std::move(name), index, ticket, data_type,
                      size, random_type, std::move(eval)),
        system_(*system) {
    DRAKE_DEMAND(system != nullptr);
    DRAKE_DEMAND(static_cast<const void*>(system) == system_base);
  }

  // This overload is chosen for the int argument if the Ref type exists,
  // otherwise there is an SFINAE failure here.
  template <typename ValueType>
  static constexpr bool is_eigen_refable_helper(
      decltype(std::declval<Eigen::Ref<ValueType>>(), int())) {
    return true;
  }

  // When the above method can't be instantiated, the int argument converts to
  // a char and this method is invoked instead.
  template <typename ValueType>
  static constexpr bool is_eigen_refable_helper(char) {
    return false;
  }

  // This is instantiated if ValueType has a copy constructor. If it is also
  // cloneable, this method still gets invoked; we prefer use of the the copy
  // constructor.
  template <
      typename ValueType,
      typename = std::enable_if_t<std::is_copy_constructible<ValueType>::value>>
  FixedInputPortValue& FixValueHelper(Context<T>* context,
                                      const ValueType& value, int, int) const {
    return context->FixInputPort(get_index(),
                                 std::make_unique<Value<ValueType>>(value));
  }

  // This is available if ValueType is cloneable, but is dispreferred if there
  // is also a copy constructor because the `int, int` args above are a better
  // match than the `int, ...` args here.
  template <typename ValueType,
            typename = std::enable_if_t<drake::is_cloneable<ValueType>::value>>
  FixedInputPortValue& FixValueHelper(Context<T>* context,
                                      const ValueType& value, int, ...) const {
    return context->FixInputPort(
        get_index(), std::make_unique<Value<ValueType>>(value.Clone()));
  }

  // This signature exists for non-copyable, non-cloneable ValueType just to
  // avoid spurious compilation errors. A static_assert above will have provided
  // a good message already; this method just needs to compile peacefully.
  template <typename ValueType>
  FixedInputPortValue& FixValueHelper(Context<T>*, const ValueType&,
                                      ...) const {
    DRAKE_UNREACHABLE();
  }

  const System<T>& system_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::InputPort)
