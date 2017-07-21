#include "drake/systems/framework/system_transmogrifier.h"

#include "drake/common/never_destroyed.h"

using std::type_index;
using std::type_info;

namespace drake {
namespace systems {

SystemTransmogrifier::SystemTransmogrifier() {}

SystemTransmogrifier::~SystemTransmogrifier() {}

void SystemTransmogrifier::Insert(
    const type_info& t_info, const type_info& u_info,
    const Func& converter) {
  const auto& key = std::make_pair(type_index(t_info), type_index(u_info));
  const auto& insert_result = funcs_.insert({key, converter});
  DRAKE_ASSERT(insert_result.second);
}

const SystemTransmogrifier::Func& SystemTransmogrifier::Find(
    const type_info& t_info, const type_info& u_info) const {
  const auto& key = std::make_pair(type_index(t_info), type_index(u_info));
  auto iter = funcs_.find(key);
  if (iter != funcs_.end()) {
    return iter->second;
  } else {
    static const never_destroyed<Func> empty{
      [](const void*) { return nullptr; }
    };
    return empty.access();
  }
}

}  // namespace systems
}  // namespace drake
