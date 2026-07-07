#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace pydrake {
namespace internal {

#ifdef PYDRAKE_USE_PYBIND11
template <typename T>
void builder_life_support_stash_impl(size_t builder_index,
    const py::detail::function_call& call, py::handle ret) {
  // Returns the handle selected by the given index. Throws if the index is
  // invalid.
  auto get_arg = [&](size_t n) -> py::handle {
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
  py::handle builder_handle = get_arg(builder_index);
  if (builder_handle.is_none()) {
    // Nothing useful to stash.
    return;
  }
  // Convert the handle to a strong reference for later stashing.
  py::object py_builder = py::cast<py::object>(builder_handle);
  // Recover the c++ pointer; pybind11 will throw if the cast can't work.
  systems::DiagramBuilder<T>* cc_builder =
      py::cast<systems::DiagramBuilder<T>*>(py_builder);
  DRAKE_ASSERT(cc_builder != nullptr);
  // Do the equivalent of stash(); we don't use the method since we've already
  // had to recover the python and c++ objects in a different order.
  BuilderLifeSupport<T>::attrs(cc_builder)
      .emplace(BuilderLifeSupport<T>::kKey, py_builder);
}
#else   // PYDRAKE_USE_NANOBIND
// TODO(rpoyner-tri): figure out if this feature can capture the function name
// for use in error messages.

template <typename T>
void builder_life_support_stash_impl(
    size_t builder_index, PyObject** args, size_t nargs, py::handle ret) {
  // Returns the handle selected by the given index. Throws if the index is
  // invalid.
  auto get_arg = [&args, &nargs, &ret](size_t n) -> py::handle {
    if (n == 0) {
      return ret;
    }
    if (n <= nargs) {
      return args[n - 1];
    }
    throw std::runtime_error(
        fmt::format("Could not activate builder_life_support_stash: index {} "
                    "is invalid",
            n));
  };
  py::handle builder_handle = get_arg(builder_index);
  if (builder_handle.is_none()) {
    // Nothing useful to stash.
    return;
  }
  // Convert the handle to a strong reference for later stashing.
  py::object py_builder = py::cast<py::object>(builder_handle);
  // For nanobind init annotations, we must ensure the ready bit is set before
  // casting to c++.
  py::inst_set_state(
      builder_handle, true, py::inst_state(builder_handle).second);
  // Recover the c++ pointer; nanobind will throw if the cast can't work.
  systems::DiagramBuilder<T>* cc_builder =
      py::cast<systems::DiagramBuilder<T>*>(py_builder);
  DRAKE_ASSERT(cc_builder != nullptr);
  // Do the equivalent of stash(); we don't use the method since we've already
  // had to recover the python and c++ objects in a different order.
  BuilderLifeSupport<T>::attrs(cc_builder)
      .emplace(BuilderLifeSupport<T>::kKey, py_builder);
}
#endif  // PYDRAKE_USE_PYBIND11

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&builder_life_support_stash_impl<T>));

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
