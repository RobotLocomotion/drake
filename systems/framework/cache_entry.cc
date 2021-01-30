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
  DRAKE_ASSERT_VOID(CheckValidAbstractValue(context, *value));

  calc_function_(context, value);
}

void CacheEntry::CheckValidAbstractValue(const ContextBase& context,
                                         const AbstractValue& proposed) const {
  const CacheEntryValue& cache_value = get_cache_entry_value(context);
  const AbstractValue& value = cache_value.PeekAbstractValueOrThrow();
  if (proposed.type_info() != value.type_info()) {
    throw std::logic_error(FormatName("Calc") +
                           "expected AbstractValue output type " +
                           value.GetNiceTypeName() + " but got " +
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
