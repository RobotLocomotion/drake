#include "drake/systems/framework/port_base.h"

#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace systems {

PortBase::PortBase(
    const char* kind_string, internal::SystemMessageInterface* owning_system,
    std::string name, int index, DependencyTicket ticket,
    PortDataType data_type, int size)
    : kind_string_(kind_string),
      owning_system_(*owning_system),
      index_(index),
      ticket_(ticket),
      data_type_(data_type),
      size_(size),
      name_(std::move(name)) {
  DRAKE_DEMAND(kind_string != nullptr);
  DRAKE_DEMAND(owning_system != nullptr);
  DRAKE_DEMAND(!name_.empty());
  if (size_ == kAutoSize) {
    throw std::domain_error("Auto-size ports are not yet implemented.");
  }
}

PortBase::~PortBase() = default;

std::string PortBase::GetFullDescription() const {
  return fmt::format(
      "{}Port[{}] ({}) of System {} ({})",
      kind_string_, index_, name_, get_system_base().GetSystemPathname(),
      NiceTypeName::RemoveNamespaces(get_system_base().GetSystemType()));
}

void PortBase::ThrowBadCast(
    const std::string& value_typename, const std::string& eval_typename) const {
  throw std::logic_error(fmt::format(
      "{}Port::Eval(): wrong value type {} specified; "
      "actual type was {} for {}.",
      kind_string_, eval_typename, value_typename, GetFullDescription()));
}

}  // namespace systems
}  // namespace drake
