#include "drake/bindings/pydrake/util/cpp_types_pybind.h"

#include <pybind11/eval.h>

namespace drake {
namespace pydrake {
namespace internal {
namespace {

// Creates a Python object that should uniquely hash for a primitive C++
// type.
py::object GetPyHash(const std::type_info& tinfo) {
  return py::make_tuple("cpp_type", tinfo.hash_code());
}

// Registers C++ type.
template <typename T>
void RegisterType(
    py::module m, py::object type_aliases, const std::string& canonical_str) {
  // Create an object that is a unique hash.
  py::object canonical = py::eval(canonical_str, m.attr("__dict__"));
  py::list aliases(1);
  aliases[0] = GetPyHash(typeid(T));
  type_aliases.attr("register")(canonical, aliases);
}

// Registers common C++ types.
void RegisterCommon(py::module m, py::object type_aliases) {
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterType<bool>(m, type_aliases, "bool");
  RegisterType<std::string>(m, type_aliases, "str");
  RegisterType<double>(m, type_aliases, "float");
  RegisterType<float>(m, type_aliases, "np.float32");
  RegisterType<int>(m, type_aliases, "int");
  RegisterType<uint32_t>(m, type_aliases, "np.uint32");
  RegisterType<int64_t>(m, type_aliases, "np.int64");
  // For supporting generic Python types.
  RegisterType<py::object>(m, type_aliases, "object");
}

}  // namespace

py::object GetTypeAliases() {
  py::module m = py::module::import("pydrake.util.cpp_types");
  py::object type_aliases = m.attr("_type_aliases");
  const char registered_check[] = "_register_common_cpp";
  if (!py::hasattr(m, registered_check)) {
    RegisterCommon(m, type_aliases);
    m.attr(registered_check) = true;
  }
  return type_aliases;
}

py::object GetPyTypeImpl(const std::type_info& tinfo) {
  py::object canonical =
      GetTypeAliases().attr("get_canonical")(GetPyHash(tinfo), false);
  if (!canonical.is_none()) {
    return canonical;
  } else {
    // If this C++ type was not explicitly registered above, then attempt
    // to get the pybind, erroring out if it's not registered.
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
