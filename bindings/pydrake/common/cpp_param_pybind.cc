#include "drake/bindings/pydrake/common/cpp_param_pybind.h"

#include "pybind11/eval.h"

namespace drake {
namespace pydrake {

// N.B. py::handle::inc_ref() and ::dec_ref() use the Py_X* variants, implying
// that they are safe to use on nullptr, thus we do not need any
// `ptr != nullptr` checks.
Object::Object(::PyObject* ptr) : ptr_(ptr) {
  inc_ref();
}
Object::~Object() {
  dec_ref();
}
Object::Object(const Object& other) : Object(other.ptr()) {}
Object::Object(Object&& other) {
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
}
Object& Object::operator=(const Object& other) {
  dec_ref();
  ptr_ = other.ptr();
  inc_ref();
  return *this;
}
Object& Object::operator=(Object&& other) {
  dec_ref();
  ptr_ = other.ptr_;
  other.ptr_ = nullptr;
  return *this;
}
void Object::inc_ref() {
  py::handle(ptr_).inc_ref();
}
void Object::dec_ref() {
  py::handle(ptr_).dec_ref();
}

namespace internal {
namespace {

// Creates a Python object that should uniquely hash for a primitive C++
// type.
py::object GetPyHash(const std::type_info& tinfo) {
  return py::make_tuple("cpp_type", tinfo.hash_code());
}

// Registers common C++ types.
void RegisterCommon() {
  // Make mappings for C++ RTTI to Python types.
  // Unfortunately, this is hard to obtain from `pybind11`.
  py::module builtins = py::module::import("builtins");
  py::module np = py::module::import("numpy");
  RegisterTypeAlias<bool>(builtins.attr("bool"));
  RegisterTypeAlias<std::string>(builtins.attr("str"));
  RegisterTypeAlias<double>(builtins.attr("float"));
  RegisterTypeAlias<float>(np.attr("float32"));
  RegisterTypeAlias<int>(builtins.attr("int"));
  RegisterTypeAlias<uint8_t>(np.attr("uint8"));
  RegisterTypeAlias<uint16_t>(np.attr("uint16"));
  RegisterTypeAlias<int16_t>(np.attr("int16"));
  RegisterTypeAlias<uint32_t>(np.attr("uint32"));
  RegisterTypeAlias<int64_t>(np.attr("int64"));
  // For supporting generic Python types.
  RegisterTypeAlias<Object>(builtins.attr("object"));
}

}  // namespace

void RegisterTypeAliasImpl(const std::type_info& tinfo, py::object canonical) {
  // Create an object that is a unique hash.
  py::object param_aliases =
      py::module::import("pydrake.common.cpp_param").attr("_param_aliases");
  py::list aliases(1);
  aliases[0] = GetPyHash(tinfo);
  param_aliases.attr("register")(canonical, aliases);
}

py::object GetParamAliases() {
  py::module m = py::module::import("pydrake.common.cpp_param");
  py::object param_aliases = m.attr("_param_aliases");
  const char registered_check[] = "_register_common_cpp";
  if (!py::hasattr(m, registered_check)) {
    RegisterCommon();
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
