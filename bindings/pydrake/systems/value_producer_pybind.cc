#include "drake/bindings/pydrake/systems/value_producer_pybind.h"

namespace drake {
namespace pydrake {

// The C++ framework uses ValueProducer::AllocateCallback to allocate storage
// for values in the Context, and that function returns a unique_ptr. Python
// cannot return unique_ptr, so we need to wrap the call to match signatures.
// The argument is the Python callback; the return value is the C++ callback.
std::function<std::unique_ptr<AbstractValue>()>
MakeCppCompatibleAllocateCallback(py::function allocate) {
  return [allocate = std::move(allocate)]() -> std::unique_ptr<AbstractValue> {
    py::gil_scoped_acquire guard;

    // Invoke the Python allocate function.
    py::object result_py = allocate();
    if (result_py.is_none()) {
      throw std::runtime_error(
          fmt::format("The allocate callback function {} must return a "
                      "pydrake.common.value.Value[...] or"
                      "pydrake.common.value.AbstractValue object, not None",
              py::repr(allocate).cast<std::string>()));
    }

    // Verify its return type.
    const AbstractValue* result_cpp;
    try {
      result_cpp = result_py.cast<const AbstractValue*>();
    } catch (const py::cast_error& e) {
      throw std::runtime_error(
          fmt::format("The allocate callback function {} must return a "
                      "pydrake.common.value.Value[...] or"
                      "pydrake.common.value.AbstractValue object, not {}",
              py::repr(allocate).cast<std::string>(),
              py::str(py::type::handle_of(result_py)).cast<std::string>()));
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
