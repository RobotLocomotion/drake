#include "drake/common/nice_type_name_override.h"

#include <mutex>
#include <typeindex>
#include <unordered_map>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {

namespace {
NiceTypeNamePtrOverride& ptr_override() {
  static never_destroyed<NiceTypeNamePtrOverride> value;
  return value.access();
}
}  // namespace

void SetNiceTypeNamePtrOverride(NiceTypeNamePtrOverride new_ptr_override) {
  DRAKE_DEMAND(ptr_override() == nullptr);
  DRAKE_DEMAND(new_ptr_override != nullptr);
  ptr_override() = new_ptr_override;
}

const NiceTypeNamePtrOverride& GetNiceTypeNamePtrOverride() {
  return ptr_override();
}

namespace {
struct TypeInfoAliasMap {
  mutable std::mutex mutex;
  std::unordered_map<std::type_index, const std::type_info*> map;
};
static TypeInfoAliasMap& GetTypeInfoAliasMapSingleton() {
  static never_destroyed<TypeInfoAliasMap> value;
  return value.access();
}
}  // namespace

void AddTypeInfoAlias(const std::type_info& alias_type,
                      const ::std::type_info* bound_type) {
  DRAKE_DEMAND(bound_type != nullptr);
  TypeInfoAliasMap& singleton = GetTypeInfoAliasMapSingleton();
  std::lock_guard<std::mutex> lock(singleton.mutex);
  singleton.map[std::type_index(alias_type)] = bound_type;
}

const std::type_info* GetTypeInfoAlias(const std::type_info* query) {
  DRAKE_DEMAND(query != nullptr);
  const TypeInfoAliasMap& singleton = GetTypeInfoAliasMapSingleton();
  std::lock_guard<std::mutex> lock(singleton.mutex);
  auto iter = singleton.map.find(std::type_index(*query));
  return (iter != singleton.map.end()) ? iter->second : query;
}

}  // namespace internal
}  // namespace drake
