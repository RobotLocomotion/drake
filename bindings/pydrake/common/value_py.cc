#include <string>

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

namespace {

// Local specialization of C++ implementation for `Value[object]`
// instantiation.
// TODO(eric.cousineau): Per discussion in #18655, should consider enforcing
// value semantics (always deepcopy, even on `set_value()`).
class PyObjectValue : public drake::Value<Object> {
 public:
  using Base = Value<Object>;
  using Base::Base;

  // Override `Clone()` to perform a deep copy on the object.
  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<PyObjectValue>(get_value().Clone());
  }

  // Override `SetFrom()` to perform a deep copy on the object.
  void SetFrom(const AbstractValue& other) override {
    get_mutable_value() = other.get_value<Object>().Clone();
  }
};

// Add instantiations of primitive types on an as-needed basis; please be
// conservative.
void AddPrimitiveValueInstantiations(py::module m) {
  AddValueInstantiation<std::string>(m);            // Value[str]
  AddValueInstantiation<bool>(m);                   // Value[bool]
  AddValueInstantiation<double>(m);                 // Value[float]
  AddValueInstantiation<Object, PyObjectValue>(m);  // Value[object]
}

}  // namespace

PYBIND11_MODULE(value, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Bindings for //common:value";
  constexpr auto& doc = pydrake_doc.drake;

  // `AddValueInstantiation` will define methods specific to `T` for
  // `Value<T>`. Since Python is nominally dynamic, these methods are
  // effectively "virtual".
  auto abstract_stub = [](const std::string& method) {
    return [method](const AbstractValue* self, py::args, py::kwargs) {
      std::string type_name = NiceTypeName::Get(*self);
      throw std::runtime_error(
          "This derived class of `AbstractValue`, `" + type_name + "`, " +
          "is not exposed to pybind11, so `" + method + "` cannot be " +
          "called. See `AddValueInstantiation` for how to bind it.");
    };
  };

  py::class_<AbstractValue> abstract_value(m, "AbstractValue");
  DefClone(&abstract_value);
  abstract_value  // BR
      .def("SetFrom", &AbstractValue::SetFrom, doc.AbstractValue.SetFrom.doc)
      .def("get_value", abstract_stub("get_value"),
          doc.AbstractValue.get_value.doc)
      .def("get_mutable_value", abstract_stub("get_mutable_value"),
          doc.AbstractValue.get_mutable_value.doc)
      .def("set_value", abstract_stub("set_value"),
          doc.AbstractValue.set_value.doc);

  // Add value instantiations for nominal data types.
  AddPrimitiveValueInstantiations(m);

  // This adds Pythonic AbstractValue.Make.
  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
