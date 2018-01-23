#include "drake/bindings/pydrake/util/cpp_param_pybind.h"

#include <pybind11/eval.h>

namespace drake {
namespace pydrake {
namespace internal {
namespace {

// Creates a Python object that should uniquely hash for a primitive C++
// type.
pybind11::object GetPyHash(const std::type_info& tinfo) {
  return pybind11::make_tuple("cpp_type", tinfo.hash_code());
}

// Registers C++ type.
template <typename T>
void RegisterType(
    pybind11::module m, pybind11::object param_aliases,
    const std::string& canonical_str) {
  // Create an object that is a unique hash.
  pybind11::object canonical =
      pybind11::eval(canonical_str, m.attr("__dict__"));
  pybind11::list aliases(1);
  aliases[0] = GetPyHash(typeid(T));
  param_aliases.attr("register")(canonical, aliases);
}

// Registers common C++ types.
void RegisterCommon(pybind11::module m, pybind11::object param_aliases) {
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  RegisterType<bool>(m, param_aliases, "bool");
  RegisterType<std::string>(m, param_aliases, "str");
  RegisterType<double>(m, param_aliases, "float");
  RegisterType<float>(m, param_aliases, "np.float32");
  RegisterType<int>(m, param_aliases, "int");
  RegisterType<uint32_t>(m, param_aliases, "np.uint32");
  RegisterType<int64_t>(m, param_aliases, "np.int64");
  // For supporting generic Python types.
  RegisterType<pybind11::object>(m, param_aliases, "object");
}

}  // namespace

pybind11::object GetParamAliases() {
  pybind11::module m = pybind11::module::import("pydrake.util.cpp_param");
  pybind11::object param_aliases = m.attr("_param_aliases");
  const char registered_check[] = "_register_common_cpp";
  if (!pybind11::hasattr(m, registered_check)) {
    RegisterCommon(m, param_aliases);
    m.attr(registered_check) = true;
  }
  return param_aliases;
}

pybind11::object GetPyParamScalarImpl(const std::type_info& tinfo) {
  pybind11::object param_aliases = GetParamAliases();
  pybind11::object py_hash = GetPyHash(tinfo);
  if (param_aliases.attr("is_aliased")(py_hash).cast<bool>()) {
    // If it's an alias, return the canonical type.
    return param_aliases.attr("get_canonical")(py_hash);
  } else {
    // This type is not aliased. Get the pybind-registered type,
    // erroring out if it's not registered.
    // WARNING: Internal API :(
    auto* info = pybind11::detail::get_type_info(tinfo);
    if (!info) {
      // TODO(eric.cousineau): Use NiceTypeName::Canonicalize(...Demangle(...))
      // once simpler dependencies are used (or something else is used to
      // justify linking in `libdrake.so`).
      const std::string name = tinfo.name();
      throw std::runtime_error(
          "C++ type is not registered in pybind: " + name);
    }
    pybind11::handle h(reinterpret_cast<PyObject*>(info->type));
    return pybind11::reinterpret_borrow<pybind11::object>(h);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
