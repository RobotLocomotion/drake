#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"

#include <fmt/format.h>

namespace drake {
namespace pydrake {

using py::handle;
using py::detail::function_call;

namespace internal {

namespace {
static constexpr char kKey[] = "_pydrake_internal_life_support";
}  // namespace

template <typename T>
void BuilderLifeSupport<T>::stash(systems::DiagramBuilder<T>* builder) {
  BuilderLifeSupport<T>::attrs(builder).emplace(kKey, py::cast(builder));
}

template <typename T>
void BuilderLifeSupport<T>::abandon(systems::DiagramBuilder<T>* builder) {
  BuilderLifeSupport<T>::attrs(builder).erase(kKey);
}

template <typename T>
string_map<std::any>& BuilderLifeSupport<T>::attrs(
    systems::DiagramBuilder<T>* builder) {
  return builder->get_mutable_life_support().attributes;
}

template <typename T>
void builder_life_support_stash_impl(size_t builder_index,
    const py::detail::function_call& call, py::handle ret) {
  // Returns the handle selected by the given index. Throws if the index is
  // invalid.
  auto get_arg = [&](size_t n) -> handle {
    if (n == 0) {
      return ret;
    }
    if (n == 1 && call.init_self) {
      return call.init_self;
    }
    if (n <= call.args.size()) {
      return call.args[n - 1];
    }
    py::pybind11_fail(
        fmt::format("Could not activate builder_life_support_stash: index {} "
                    "is invalid for function '{}'",
            n, call.func.name));
  };
  handle py_builder = get_arg(builder_index);
  if (py_builder.is_none()) {
    // Nothing useful to stash.
    return;
  }
  systems::DiagramBuilder<T>* cc_builder =
      py::cast<systems::DiagramBuilder<T>*>(py_builder);
  if (cc_builder == nullptr) {
    // Nothing useful to stash.
    return;
  }
  BuilderLifeSupport<T>::attrs(cc_builder).emplace(kKey, py_builder);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::pydrake::internal::BuilderLifeSupport);

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&drake::pydrake::internal::builder_life_support_stash_impl<T>));
