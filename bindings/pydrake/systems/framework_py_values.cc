#include "drake/bindings/pydrake/systems/framework_py_values.h"

#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/eigen_pybind.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/supervector.h"

using std::string;

namespace drake {
namespace pydrake {

using pysystems::AddValueInstantiation;
using pysystems::DefClone;

void DefineFrameworkPyValues(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    // Value types.
    DefineTemplateClassWithDefault<VectorBase<T>>(
      m, "VectorBase", GetPyParam<T>())
      .def("CopyToVector", &VectorBase<T>::CopyToVector)
      .def("SetAtIndex", &VectorBase<T>::SetAtIndex)
      .def("SetFromVector", &VectorBase<T>::SetFromVector)
      .def("size", &VectorBase<T>::size);

    // TODO(eric.cousineau): Make a helper function for the Eigen::Ref<>
    // patterns.
    auto basic_vector =
        DefineTemplateClassWithDefault<BasicVector<T>, VectorBase<T>>(
            m, "BasicVector", GetPyParam<T>());
    DefClone(&basic_vector);
    basic_vector
      // N.B. Place `init<VectorX<T>>` `init<int>` so that we do not implicitly
      // convert scalar-size `np.array` objects to `int` (since this is normally
      // permitted).
      .def(py::init<VectorX<T>>())
      .def(py::init<int>())
      .def("get_value",
          [](const BasicVector<T>* self) -> Eigen::Ref<const VectorX<T>> {
            return self->get_value();
          }, py_reference_internal)
      // TODO(eric.cousineau): Remove this once `get_value` is changed, or
      // reference semantics are changed for custom dtypes.
      .def("_get_value_copy",
          [](const BasicVector<T>* self) -> VectorX<T> {
            return self->get_value();
          })
      .def("get_mutable_value",
          [](BasicVector<T>* self) -> Eigen::Ref<VectorX<T>> {
            return self->get_mutable_value();
          }, py_reference_internal)
       .def("GetAtIndex", [](BasicVector<T>* self, int index) -> T& {
         return self->GetAtIndex(index);
       }, py_reference_internal);

    DefineTemplateClassWithDefault<Supervector<T>, VectorBase<T>>(
        m, "Supervector", GetPyParam<T>());

    DefineTemplateClassWithDefault<Subvector<T>, VectorBase<T>>(
        m, "Subvector", GetPyParam<T>());
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});

  // `AddValueInstantiation` will define methods specific to `T` for
  // `Value<T>`. Since Python is nominally dynamic, these methods are
  // effectively "virtual".
  auto abstract_stub = [](const std::string& method) {
    return [method](const AbstractValue* self, py::args, py::kwargs) {
      string type_name = NiceTypeName::Get(*self);
      throw std::runtime_error(
          "This derived class of `AbstractValue`, `" + type_name + "`, " +
          "is not exposed to pybind11, so `" + method + "` cannot be " +
          "called. See `AddValueInstantiation` for how to bind it.");
    };
  };

  py::class_<AbstractValue> abstract_value(m, "AbstractValue");
  DefClone(&abstract_value);
  abstract_value
    // Only bind the exception variant, `SetFromOrThrow`, for use in Python.
    // Otherwise, a user could encounter undefind behavior via `SetFrom`.
    .def("SetFrom", &AbstractValue::SetFromOrThrow)
    .def("get_value", abstract_stub("get_value"))
    .def("get_mutable_value", abstract_stub("get_mutable_value"))
    .def("set_value", abstract_stub("set_value"));

  // Add `Value<std::string>` instantiation (visible in Python as `Value[str]`).
  AddValueInstantiation<string>(m);

  // Add `Value[object]` instantiation.
  // N.B. If any code explicitly uses `Value<py::object>` for whatever reason,
  // then this should turn into a specialization of `Value<>`, rather than an
  // extension.
  class PyObjectValue : public Value<py::object> {
   public:
    using Base = Value<py::object>;
    using Base::Base;
    // Override `Value<py::object>::Clone()` to perform a deep copy on the
    // object.
    std::unique_ptr<AbstractValue> Clone() const override {
      py::object py_copy = py::module::import("copy").attr("deepcopy");
      return std::make_unique<PyObjectValue>(py_copy(get_value()));
    }
  };
  AddValueInstantiation<py::object, PyObjectValue>(m);

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
      });
}

}  // namespace pydrake
}  // namespace drake
