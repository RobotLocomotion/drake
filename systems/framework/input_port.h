#pragma once

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

  /** Returns true iff this port is connected.  Beware that at the moment, this
  could be an expensive operation, because the value is brought up-to-date as
  part of this operation. */
  bool HasValue(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(get_system_base().ThrowIfContextNotCompatible(context));
    return DoEvalOptional(context);
  }

  /** Returns a reference to the System that owns this input port. Note that
  for a Diagram input port this will be the Diagram, not the LeafSystem whose
  input port was exported. */
  const System<T>& get_system() const { return system_; }

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

  const System<T>& system_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::InputPort)
