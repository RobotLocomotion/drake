#include "drake/systems/framework/cache_entry.h"

#include <memory>
#include <sstream>
#include <typeinfo>

#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

std::unique_ptr<AbstractValue> CacheEntry::Allocate(
    const ContextBase& context) const {
  DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));
  std::unique_ptr<AbstractValue> value = alloc_function_(context);
  if (value == nullptr) {
    throw std::logic_error(
        "CacheEntry::Allocate(): allocator returned a nullptr for " +
        GetCacheEntryIdString());
  }
  return value;
}

void CacheEntry::Calc(const ContextBase& context,
                      AbstractValue* value) const {
  DRAKE_DEMAND(value != nullptr);
  DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));
  DRAKE_ASSERT_VOID(CheckValidAbstractValue(context, *value));

  calc_function_(context, value);
}

std::string CacheEntry::GetCacheEntryIdString() const {
  std::ostringstream oss;
  oss << "cache entry " << cache_index() << " (" << description_ << ") of "
      << this->get_system().GetSystemIdString();
  return oss.str();
}

void CacheEntry::CheckValidAbstractValue(
    const ContextBase& context, const AbstractValue& proposed) const {
  auto good_ptr = Allocate(context);  // Very expensive!
  const AbstractValue& good = *good_ptr;
  if (typeid(proposed) != typeid(good)) {
    std::ostringstream oss;
    oss << "CacheEntry::Calc(): expected AbstractValue output type "
        << NiceTypeName::Get(good) << " but got "
        << NiceTypeName::Get(proposed)
        << " for " << GetCacheEntryIdString();
    throw std::logic_error(oss.str());
  }
}

}  // namespace systems
}  // namespace drake
