#include <string>

#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace pydrake {

namespace {

// Local specialization of C++ implementation for `Value[object]`
// instantiation.
class PyObjectValue : public drake::Value<Object> {
 public:
  using Base = Value<Object>;
  using Base::Base;
  // Override `Clone()` to perform a deep copy on the object.
  std::unique_ptr<AbstractValue> Clone() const override {
    py::object py_copy = py::module::import("copy").attr("deepcopy");
    py::object copied = py_copy(get_value().to_pyobject<py::object>());
    return std::make_unique<PyObjectValue>(Object::from_pyobject(copied));
  }
};

// Add instantiations of primitive types on an as-needed basis; please be
// conservative.
void AddPrimitiveValueInstantiations(py::module m) {
  AddValueInstantiation<std::string>(m);            // Value[str]
  AddValueInstantiation<bool>(m);                   // Value[bool]
  AddValueInstantiation<Object, PyObjectValue>(m);  // Value[object]
}

template <typename T, typename Class = drake::Value<T>>
py::class_<Class> AddEigenValueInstantiation(py::module scope) {
  py::module py_common = py::module::import("pydrake.common.value");
  py::class_<Class, drake::AbstractValue> py_class(
      scope, TemporaryClassName<Class>().c_str());
  // Register instantiation, but do not bind emplace constructor.
  AddTemplateClass(py_common, "Value", py_class, GetPyParam<T>());
  py_class  // BR
      .def(py::init())
      .def(py::init<const T&>())
      .def("get_value", &Class::get_value, py_reference_internal)
      .def(
          "get_mutable_value", &Class::get_mutable_value, py_reference_internal)
      .def("set_value", &Class::set_value);
  return py_class;
}

constexpr char kEigenPlaceholderDoc[] = R"""(
Placeholder to refer to C++ Eigen Type. This class cannot be instantiated.
)""";

template <typename T>
void BindEigenValueInstantiations(py::module m, T = {}) {
  py::handle create_placeholder_cls = m.attr("_create_placeholder_cls");
  py::handle vector_template = m.attr("VectorX");
  py::object vector_cls = create_placeholder_cls(
      vector_template, GetPyParam<T>(), kEigenPlaceholderDoc);
  RegisterTypeAlias<VectorX<T>>(vector_cls);
  AddEigenValueInstantiation<VectorX<T>>(m);

  py::handle matrix_template = m.attr("MatrixX");
  py::object matrix_cls = create_placeholder_cls(
      matrix_template, GetPyParam<T>(), kEigenPlaceholderDoc);
  RegisterTypeAlias<MatrixX<T>>(matrix_cls);
  AddEigenValueInstantiation<MatrixX<T>>(m);
}

void BindAllEigenValueInstantiations(py::module m) {
  type_visit([m](auto dummy) { BindEigenValueInstantiations(m, dummy); },
      CommonScalarPack{});
}

}  // namespace

PYBIND11_MODULE(value, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Bindings for //common:value";
  constexpr auto& doc = pydrake_doc.drake;

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.symbolic");

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

  py::object py_type_func = py::eval("type");
  py::object py_object_type = py::eval("object");
  // `Value` was defined by the first call to `AddValueInstantiation`.
  py::object py_value_template = m.attr("Value");
  abstract_value.def_static(
      "Make",
      [py_type_func, py_value_template, py_object_type](py::object value) {
        // Try to infer type from the object. If that does not work, just return
        // `Value[object]`.
        py::object py_type = py_type_func(value);
        py::tuple py_result =
            py_value_template.attr("get_instantiation")(py_type, false);
        py::object py_value_class = py_result[0];
        if (py_value_class.is_none()) {
          py_value_class = py_value_template[py_object_type];
        }
        return py_value_class(value);
      },
      doc.AbstractValue.Make.doc);

  // For supporting mapping between Eigen and NumPy, namely for abstract
  // values.
  m.def("_bind_all_eigen_value_instantiations",
      [m]() { BindAllEigenValueInstantiations(m); });

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
