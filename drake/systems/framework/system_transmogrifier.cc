#include "drake/systems/framework/system_transmogrifier.h"

#include <map>
#include <typeindex>
#include <utility>

#include "drake/common/never_destroyed.h"

using std::type_index;
using std::type_info;

namespace drake {
namespace systems {

struct SystemTransmogrifier::Impl {
  std::map<std::pair<type_index, type_index>, Func> converter_functions;
};

SystemTransmogrifier::SystemTransmogrifier()
    : impl_(std::make_unique<Impl>()) {}

SystemTransmogrifier::SystemTransmogrifier(const SystemTransmogrifier& other)
    : impl_(std::make_unique<Impl>(*other.impl_)) {}

SystemTransmogrifier&
SystemTransmogrifier::operator=(const SystemTransmogrifier& other) {
  impl_ = std::make_unique<Impl>(*other.impl_);
  return *this;
}

SystemTransmogrifier::~SystemTransmogrifier() {}

void SystemTransmogrifier::Insert(
    const type_info& t_info, const type_info& u_info,
    const Func& converter) const {
  const auto& key = std::make_pair(type_index(t_info), type_index(u_info));
  const auto& insert_result =
      impl_->converter_functions.insert({key, converter});
  DRAKE_ASSERT(insert_result.second);
}

const SystemTransmogrifier::Func& SystemTransmogrifier::Find(
    const type_info& t_info, const type_info& u_info) const {
  const auto& key = std::make_pair(type_index(t_info), type_index(u_info));
  auto iter = impl_->converter_functions.find(key);
  if (iter != impl_->converter_functions.end()) {
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
