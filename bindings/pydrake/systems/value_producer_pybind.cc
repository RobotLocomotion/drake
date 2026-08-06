#include "drake/bindings/pydrake/systems/value_producer_pybind.h"

#include <string>
#include <utility>

namespace drake {
namespace pydrake {

std::function<std::unique_ptr<AbstractValue>()>
MakeCppCompatibleAllocateCallback(py::callable allocate) {
  return [allocate = std::move(allocate)]() -> std::unique_ptr<AbstractValue> {
    py::gil_scoped_acquire guard;

    // Invoke the Python allocate function.
    py::object result_py = allocate();
    if (result_py.is_none()) {
      throw std::runtime_error(
          fmt::format("The allocate callback function {} must return a "
                      "pydrake.common.value.Value[...] or"
                      "pydrake.common.value.AbstractValue object, not None",
              py::cast<std::string>(py::repr(allocate))));
    }

    // Verify its return type.
    const AbstractValue* result_cpp;
    try {
      result_cpp = py::cast<const AbstractValue*>(result_py);
    } catch (const py::cast_error& e) {
      throw std::runtime_error(
          fmt::format("The allocate callback function {} must return a "
                      "pydrake.common.value.Value[...] or"
                      "pydrake.common.value.AbstractValue object, not {}",
              py::cast<std::string>(py::repr(allocate)),
              py::cast<std::string>(py::str(py::type::handle_of(result_py)))));
    }

    // Our signature requires returning a unique_ptr; the only way we can do
    // that is via cloning.
    return result_cpp->Clone();
  };
}

std::function<void(const systems::ContextBase&, AbstractValue*)>
MakeCppCompatibleCalcCallback(
    std::function<void(py::object, py::object)> calc) {
  return [calc = std::move(calc)](const systems::ContextBase& context_cpp,
             AbstractValue* output_cpp) -> void {
    py::gil_scoped_acquire guard;
    py::object context_py = py::cast(context_cpp, py_rvp::reference);
    py::object output_py = py::cast(output_cpp, py_rvp::reference);
    calc(context_py, output_py);
  };
}

}  // namespace pydrake
}  // namespace drake
