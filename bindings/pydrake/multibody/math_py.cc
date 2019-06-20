#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_velocity.h"

namespace drake {
namespace pydrake {

// Binds any child classes of the `SpatialVector` mixin.
template <typename T, typename PyClass>
void BindSpatialVectorMixin(PyClass* pcls) {
  constexpr auto& doc = pydrake_doc.drake.multibody;
  using Class = typename PyClass::type;
  auto& cls = *pcls;
  cls  // BR
      .def("rotational",
          [](const Class* self) -> const Vector3<T> {
            return self->rotational();
          },
          doc.SpatialVector.rotational.doc_0args_const)
      .def("translational",
          [](const Class* self) -> const Vector3<T> {
            return self->translational();
          },
          doc.SpatialVector.translational.doc_0args_const)
      .def("SetZero", &Class::SetZero, doc.SpatialVector.SetZero.doc)
      .def("get_coeffs", [](const Class& self) { return self.get_coeffs(); },
          doc.SpatialVector.get_coeffs.doc_0args_const)
      .def_static("Zero", &Class::Zero, doc.SpatialVector.Zero.doc);
}

namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "Bindings for multibody math.";

  {
    using Class = SpatialVelocity<T>;
    constexpr auto& cls_doc = doc.SpatialVelocity;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialVelocity", param, cls_doc.doc);
    BindSpatialVectorMixin<T>(&cls);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("w"), py::arg("v"), cls_doc.ctor.doc_2args)
        .def(py::init<const Vector6<T>&>(), py::arg("V"),
            cls_doc.ctor.doc_1args);
  }
  {
    using Class = SpatialAcceleration<T>;
    constexpr auto& cls_doc = doc.SpatialAcceleration;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialAcceleration", param, cls_doc.doc);
    BindSpatialVectorMixin<T>(&cls);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("alpha"), py::arg("a"), cls_doc.ctor.doc_2args)
        .def(py::init<const Vector6<T>&>(), py::arg("A"),
            cls_doc.ctor.doc_1args);
  }
  {
    using Class = multibody::SpatialForce<T>;
    constexpr auto& cls_doc = doc.SpatialForce;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialForce", param, cls_doc.doc);
    BindSpatialVectorMixin<T>(&cls);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("tau"), py::arg("f"), cls_doc.ctor.doc_2args)
        .def(py::init<const Vector6<T>&>(), py::arg("F"),
            cls_doc.ctor.doc_1args);
  }
}
}  // namespace

PYBIND11_MODULE(math, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  m.doc() = "Bindings for multibody math.";
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
