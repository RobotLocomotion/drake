#include "drake/systems/framework/cache_entry.h"

#include <exception>
#include <memory>
#include <typeinfo>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace systems {

CacheEntry::CacheEntry(
    const internal::SystemMessageInterface* owning_system, CacheIndex index,
    DependencyTicket ticket, std::string description,
    AllocCallback alloc_function, CalcCallback calc_function,
    std::set<DependencyTicket> prerequisites_of_calc)
    : owning_system_(owning_system),
      cache_index_(index),
      ticket_(ticket),
      description_(std::move(description)),
      alloc_function_(std::move(alloc_function)),
      calc_function_(std::move(calc_function)),
      prerequisites_of_calc_(
          std::move(prerequisites_of_calc)) {
  DRAKE_DEMAND(index.is_valid() && ticket.is_valid());
  DRAKE_DEMAND(owning_system && alloc_function_ && calc_function_);

  if (prerequisites_of_calc_.empty()) {
    throw std::logic_error(FormatName("CacheEntry") +
        "Cannot create a CacheEntry with an empty prerequisites list. If the "
        "Calc() function really has no dependencies, list 'nothing_ticket()' "
        "as its sole prerequisite.");
  }
}

std::unique_ptr<AbstractValue> CacheEntry::Allocate() const {
  std::unique_ptr<AbstractValue> value = alloc_function_();
  if (value == nullptr) {
    throw std::logic_error(FormatName("Allocate") +
                           "allocator returned a nullptr.");
  }
  return value;
}

void CacheEntry::Calc(const ContextBase& context,
                      AbstractValue* value) const {
  DRAKE_DEMAND(value != nullptr);
  DRAKE_ASSERT_VOID(owning_system_->ValidateContext(context));
  DRAKE_ASSERT_VOID(CheckValidAbstractValue(*value));

  calc_function_(context, value);
}

// See OutputPort::CheckValidOutputType; treat both methods similarly.
void CacheEntry::CheckValidAbstractValue(const AbstractValue& proposed) const {
  // TODO(sherm1) Consider whether we can depend on there already being an
  //              object of this type in the context's CacheEntryValue so we
  //              wouldn't have to allocate one here. If so could also store
  //              a precomputed type_index there for further savings. Would
  //              need to pass in a ContextBase.
  auto good_ptr = Allocate();  // Very expensive!
  const AbstractValue& good = *good_ptr;
  if (proposed.type_info() != good.type_info()) {
    throw std::logic_error(FormatName("Calc") +
                           "expected AbstractValue output type " +
                           good.GetNiceTypeName() + " but got " +
                           proposed.GetNiceTypeName() + ".");
  }
}

std::string CacheEntry::FormatName(const char* api) const {
  return "System '" + owning_system_->GetSystemPathname() + "' (" +
      NiceTypeName::RemoveNamespaces(owning_system_->GetSystemType()) +
      "): CacheEntry[" + std::to_string(cache_index_) + "](" +
      description() + ")::" + api + "(): ";
}

}  // namespace systems
}  // namespace drake
