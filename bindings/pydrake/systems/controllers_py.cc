#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/wrap_function.h"
#include "drake/systems/controllers/dynamic_programming.h"

namespace drake {
namespace pydrake {

namespace {

template <typename T, typename = void>
struct wrap_ptr : public wrap_arg_default<T> {};

template <typename T>
struct wrap_ptr<const systems::Context<T>&> {
  using Type = systems::Context<T>;
  static auto wrap(const Type& arg) { return &arg; }
  static auto unwrap(const Type* arg_wrapped) { return *arg_wrapped; }
};

// Ensures that `const Context<T>&` is wrapped with `const Context<T>*`.
// TODO(eric.cousineau): Replace this with general wrappper, place in
// `pydrake_pybind` or somewhere related.
template <typename Func>
auto WrapPtr(Func&& func) {
  return WrapFunction<wrap_ptr>(std::forward<Func>(func));
}

}  // namespace

PYBIND11_MODULE(controllers, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::controllers;

  py::module::import("pydrake._math");
  py::module::import("pydrake.systems.primitives");

  py::class_<DynamicProgrammingOptions>(m, "DynamicProgrammingOptions")
      .def(py::init<>())
      .def_readwrite("discount_factor",
                     &DynamicProgrammingOptions::discount_factor)
      .def_readwrite("state_indices_with_periodic_boundary_conditions",
                     &DynamicProgrammingOptions::
                         state_indices_with_periodic_boundary_conditions)
      .def_readwrite("convergence_tol",
                     &DynamicProgrammingOptions::convergence_tol)
      .def_readwrite("visualization_callback",
                     &DynamicProgrammingOptions::visualization_callback);

  m.def("FittedValueIteration", WrapPtr(&FittedValueIteration));
}

}  // namespace pydrake
}  // namespace drake
