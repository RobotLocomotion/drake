#include "drake/bindings/pydrake/common/cpp_param_pybind.h"

#include "pybind11/eval.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

// Creates a Python object that should uniquely hash for a primitive C++
// type.
py::object GetPyHash(const std::type_info& tinfo) {
  // N.B. Using `::hash_code() for `py::` symbols on Mac may fail depending on
  // import order. This was encountered in the failure documented by #8704. The
  // code change introduced was the usage of `GetPyParam()` in
  // `framework_py*.cc`, which caused a different `py::object` typeid to be
  // used, making `analysis_py.cc` to fail when it tried to obtain its type.
  return py::make_tuple("cpp_type", tinfo.name());
}

// Registers C++ type.
template <typename T>
void RegisterType(
    py::module m, py::object param_aliases, const std::string& canonical_str) {
  // Create an object that is a unique hash.
  py::object canonical = py::eval(canonical_str, m.attr("__dict__"));
  py::list aliases(1);
  aliases[0] = GetPyHash(typeid(T));
  param_aliases.attr("register")(canonical, aliases);
}

// Registers common C++ types.
void RegisterCommon(py::module m, py::object param_aliases) {
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterType<bool>(m, param_aliases, "bool");
  RegisterType<std::string>(m, param_aliases, "str");
  RegisterType<double>(m, param_aliases, "float");
  RegisterType<float>(m, param_aliases, "np.float32");
  RegisterType<int>(m, param_aliases, "int");
  RegisterType<uint8_t>(m, param_aliases, "np.uint8");
  RegisterType<uint16_t>(m, param_aliases, "np.uint16");
  RegisterType<int16_t>(m, param_aliases, "np.int16");
  RegisterType<uint32_t>(m, param_aliases, "np.uint32");
  RegisterType<int64_t>(m, param_aliases, "np.int64");
  // For supporting generic Python types.
  RegisterType<py::object>(m, param_aliases, "object");
}

}  // namespace

py::object GetParamAliases() {
  py::module m = py::module::import("pydrake.common.cpp_param");
  py::object param_aliases = m.attr("_param_aliases");
  const char registered_check[] = "_register_common_cpp";
  if (!py::hasattr(m, registered_check)) {
    RegisterCommon(m, param_aliases);
    m.attr(registered_check) = true;
  }
  return param_aliases;
}

py::object GetPyParamScalarImpl(const std::type_info& tinfo) {
  py::object param_aliases = GetParamAliases();
  py::object py_hash = GetPyHash(tinfo);
  if (param_aliases.attr("is_aliased")(py_hash).cast<bool>()) {
    // If it's an alias, return the canonical type.
    return param_aliases.attr("get_canonical")(py_hash);
  } else {
    // This type is not aliased. Get the pybind-registered type,
    // erroring out if it's not registered.
    // WARNING: Internal API :(
    auto* info = py::detail::get_type_info(tinfo);
    if (!info) {
      // TODO(eric.cousineau): Use NiceTypeName::Canonicalize(...Demangle(...))
      // once simpler dependencies are used (or something else is used to
      // justify linking in `libdrake.so`).
      const std::string name = tinfo.name();
      throw std::runtime_error("C++ type is not registered in pybind: " + name);
    }
    py::handle h(reinterpret_cast<PyObject*>(info->type));
    return py::reinterpret_borrow<py::object>(h);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
