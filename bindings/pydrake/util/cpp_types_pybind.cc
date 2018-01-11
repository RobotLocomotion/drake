#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

#include <pybind11/eval.h>

const char kModule[] = "pydrake.util.cpp_types";

namespace drake {
namespace pydrake {
namespace internal {

py::object GetTypeRegistry() {
  auto m = py::module::import(kModule);
  return m.attr("_type_registry");
}

void RegisterTypeImpl(const std::string& py_type_str, size_t cpp_type) {
  GetTypeRegistry().attr("register_cpp")(py::eval(py_type_str), cpp_type);
}

py::object GetPyTypeImpl(const std::type_info& tinfo) {
  py::object py_type = GetTypeRegistry().attr("get_type_canonical_from_cpp")(
      tinfo.hash_code());
  if (!py_type.is_none()) {
    return py_type;
  } else {
    // Get from pybind11-registered types.
    // WARNING: Internal API :(
    auto* info = py::detail::get_type_info(tinfo);
    if (!info) {
      throw std::runtime_error("Unknown custom type");
    }
    py::handle h(reinterpret_cast<PyObject*>(info->type));
    return py::reinterpret_borrow<py::object>(h);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
