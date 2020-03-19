#pragma once

#include <string>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

/** A PortBase is base class for System ports; users will typically use the the
 InputPort<T> or OutputPort<T> types, not this base class. */
class PortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PortBase)

  virtual ~PortBase();

  /** Get port name. */
  const std::string& get_name() const { return name_; }

  /** Returns a verbose human-readable description of port. This is useful for
  error messages or debugging. */
  std::string GetFullDescription() const;

  /** Returns the port data type. */
  PortDataType get_data_type() const { return data_type_; }

  /** Returns the fixed size expected for a vector-valued port. Not
  meaningful for abstract-valued ports. */
  int size() const { return size_; }

#ifndef DRAKE_DOXYGEN_CXX
  // Returns a reference to the system that owns this port. Note that for a
  // diagram port this will be the diagram, not the leaf system whose port was
  // exported.
  const internal::SystemMessageInterface& get_system_interface() const {
    return owning_system_;
  }
#endif

  /** (Advanced.) Returns the DependencyTicket for this port within the owning
  System. */
  DependencyTicket ticket() const {
    return ticket_;
  }

 protected:
  /** Provides derived classes the ability to set the base
  class members at construction.

  @param kind_string
    Either "Input" or "Output", depending on the kind of subclass.
  @param owning_system
    The System that owns this port.
  @param name
    A name for the port. Port names should be non-empty and unique within a
    single System.
  @param index
    The index to be assigned to this port.  Input ports and output ports each
    have their own pool of indices (InputPortIndex and OutputPortIndex); this
    is just that TypeSafeIndex passed as a bare int.
  @param ticket
    The DependencyTicket to be assigned to this port.
  @param data_type
    Whether the port described is vector- or abstract-valued.
  @param size
    If the port described is vector-valued, the number of elements, or kAutoSize
    if determined by connections. Ignored for abstract-valued ports.
  */
  PortBase(
      const char* kind_string, internal::SystemMessageInterface* owning_system,
      std::string name, int index, DependencyTicket ticket,
      PortDataType data_type, int size);

  /** Returns the index of this port within the owning System (i.e., an
  InputPortIndex or OutputPortIndex, but as a bare integer). For a Diagram,
  this will be the index within the Diagram, _not_ the index within the
  LeafSystem whose output port was forwarded. */
  int get_int_index() const { return index_; }

  /** Returns get_system_interface(), but without the const. */
  internal::SystemMessageInterface& get_mutable_system_interface() {
    return owning_system_;
  }

  /** Pull a value of a given type from an abstract value or issue a nice
  message if the type is not correct. */
  template <typename ValueType>
  const ValueType& PortEvalCast(const AbstractValue& abstract) const;

  /** Downcast a basic vector to a more specific subclass (e.g., as generated
  by //tools/vector_gen) or issue a nice message if the type is not correct. */
  template <typename ValueType, typename T>
  const ValueType& PortEvalCast(const BasicVector<T>& basic) const;

  /** Reports that the user provided a bad ValueType argument to Eval. */
  template <typename ValueType>
  [[noreturn]] const ValueType& ThrowBadCast(
      const AbstractValue& abstract) const {
    ThrowBadCast(abstract.GetNiceTypeName(), NiceTypeName::Get<ValueType>());
  }

  /** Reports that the user provided a bad ValueType argument to Eval. */
  template <typename ValueType, typename T>
  [[noreturn]] const ValueType& ThrowBadCast(
      const BasicVector<T>& basic) const {
    ThrowBadCast(NiceTypeName::Get(basic), NiceTypeName::Get<ValueType>());
  }

  /** Reports that the user provided a bad ValueType argument to Eval.  The
  value_typename is the type of the port's current value; the eval_typename is
  the type the user asked for. */
  [[noreturn]] void ThrowBadCast(
      const std::string& value_typename,
      const std::string& eval_typename) const;

 private:
  // "Input" or "Output" (used for human-readable debugging strings).
  const char* const kind_string_;

  // Associated System and System resources.
  internal::SystemMessageInterface& owning_system_;
  const int index_;
  const DependencyTicket ticket_;

  // Port details.
  const PortDataType data_type_;
  const int size_;
  const std::string name_;
};

// Keep this inlineable.  Error reporting should happen in a separate method.
template <typename ValueType>
const ValueType& PortBase::PortEvalCast(const AbstractValue& abstract) const {
  const ValueType* const value = abstract.maybe_get_value<ValueType>();
  if (!value) {
    ThrowBadCast<ValueType>(abstract);
  }
  return *value;
}

// Keep this inlineable.  Error reporting should happen in a separate method.
template <typename ValueType, typename T>
const ValueType& PortBase::PortEvalCast(const BasicVector<T>& basic) const {
  const ValueType* const value = dynamic_cast<const ValueType*>(&basic);
  if (!value) {
    ThrowBadCast<ValueType>(basic);
  }
  return *value;
}

}  // namespace systems
}  // namespace drake
