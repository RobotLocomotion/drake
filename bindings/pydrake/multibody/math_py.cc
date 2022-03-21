#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_deprecated.h"
#include "drake/multibody/math/spatial_algebra.h"

#pragma GCC diagnostic push
// Similar to `symbolic_py.cc`, we must suppress `-Wself-assign-overloaded` to
/// use operators for Apple's clang>=10. However, different than
// `symbolic_py.cc`, we must also enable this for clang 9 on Bionic, as is
// triggered on `py::self -= py::self` for some reason.
// It is fine to use this at a file-wide scope since in practice we only
// encounter these warnings in bindings due to pybind11's operators.
#if (__clang__) && (__clang_major__ >= 9)
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

namespace drake {
namespace pydrake {

using math::RotationMatrix;

// Binds any child classes of the `SpatialVector` mixin.
template <typename T, typename PyClass>
void BindSpatialVectorMixin(PyClass* pcls) {
  constexpr auto& doc = pydrake_doc.drake.multibody;
  using Class = typename PyClass::type;
  auto& cls = *pcls;
  cls  // BR
      .def(py::init([]() {
        // See #14086 for more details.
        Class out;
        out.SetNaN();
        return out;
      }),
          R"""(
      Constructs to all NaNs.

      Note:
          This is different from C++, which in Release builds may leave memory
          uninitialized. In pydrake, the function call overhead already trumps
          any overhead from NAN-initialization, so we err on the side of
          safety.
      )""")
      .def(
          "rotational",
          [](const Class* self) -> const Vector3<T> {
            return self->rotational();
          },
          doc.SpatialVector.rotational.doc_0args_const)
      .def(
          "translational",
          [](const Class* self) -> const Vector3<T> {
            return self->translational();
          },
          doc.SpatialVector.translational.doc_0args_const)
      .def("SetZero", &Class::SetZero, doc.SpatialVector.SetZero.doc)
      .def(
          "get_coeffs", [](const Class& self) { return self.get_coeffs(); },
          doc.SpatialVector.get_coeffs.doc_0args_const)
      .def_static("Zero", &Class::Zero, doc.SpatialVector.Zero.doc)
      .def(-py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(py::self *= T{})
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self * T{})
      .def(T{} * py::self)
      .def(
          "Rotate",
          [](const Class& self, RotationMatrix<T>& R_FE) {
            return R_FE * self;
          },
          py::arg("R_FE"),
          R"""(
          Provides a Python-only implementation of rotating / re-expressing a
          spatial vector.

          Note:
              This is done because defining ``__rmatmul__`` on this class does
              not disambiguate against the definitions of
              ``RotationMatrix.__matmul__``.
          )""")
      .def(py::pickle([](const Class& self) { return self.get_coeffs(); },
          [](const Vector6<T>& coeffs) { return Class(coeffs); }));
  DefCopyAndDeepCopy(&cls);
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
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("w"), py::arg("v"), cls_doc.ctor.doc_2args)
        .def(
            py::init<const Vector6<T>&>(), py::arg("V"), cls_doc.ctor.doc_1args)
        .def("Shift", &Class::Shift, py::arg("offset"), cls_doc.Shift.doc);
    constexpr char doc_Shift_deprecatedArgName[] =
        "The keyword argument (kwarg) has been renamed from"
        " Shift(p_BqBq_E) to"
        " Shift(offset)."
        " Deprecated kwarg will be unavailable after 2022-06-01.";
    cls.def("Shift", WrapDeprecated(doc_Shift_deprecatedArgName, &Class::Shift),
           py::arg("p_BpBq_E"), doc_Shift_deprecatedArgName)
        .def("ComposeWithMovingFrameVelocity",
            &Class::ComposeWithMovingFrameVelocity,
            py::arg("position_of_moving_frame"),
            py::arg("velocity_of_moving_frame"),
            cls_doc.ComposeWithMovingFrameVelocity.doc);
    constexpr char doc_ComposeWithMovingFrameVelocity_deprecatedArgName[] =
        "The keyword arguments (kwargs) have been renamed from"
        " ComposeWithMovingFrameVelocity(p_PoBo_E, V_PB_E) to"
        " ComposeWithMovingFrameVelocity(position_of_moving_frame,"
        " velocity_of_moving_frame)."
        " Deprecated kwargs will be unavailable after 2022-06-01.";
    cls.def("ComposeWithMovingFrameVelocity",
        WrapDeprecated(doc_ComposeWithMovingFrameVelocity_deprecatedArgName,
            &Class::ComposeWithMovingFrameVelocity),
        py::arg("p_PoBo_E"), py::arg("V_PB_E"),
        doc_ComposeWithMovingFrameVelocity_deprecatedArgName);
    cls.def("dot",
        overload_cast_explicit<T, const SpatialForce<T>&>(&Class::dot),
        py::arg("force"), cls_doc.dot.doc_1args_force);
    constexpr char doc_dot_deprecatedArgNameF_Bp_E[] =
        "The keyword argument (kwarg) has been renamed from"
        " dot(F_Bp_E) to"
        " dot(force)."
        " Deprecated kwarg will be unavailable after 2022-06-01.";
    cls.def("dot",
        WrapDeprecated(doc_dot_deprecatedArgNameF_Bp_E,
            overload_cast_explicit<T, const SpatialForce<T>&>(&Class::dot)),
        py::arg("F_Bp_E"), doc_dot_deprecatedArgNameF_Bp_E);
    cls.def("dot",
        overload_cast_explicit<T, const SpatialMomentum<T>&>(&Class::dot),
        py::arg("momentum"), cls_doc.dot.doc_1args_momentum);
    constexpr char doc_dot_deprecatedArgNameL_WBp_E[] =
        "The keyword argument (kwarg) has been renamed from"
        " dot(L_WBp_E) to"
        " dot(momentum)."
        " Deprecated kwarg will be unavailable after 2022-06-01.";
    cls.def("dot",
        WrapDeprecated(doc_dot_deprecatedArgNameL_WBp_E,
            overload_cast_explicit<T, const SpatialMomentum<T>&>(&Class::dot)),
        py::arg("L_WBp_E"), doc_dot_deprecatedArgNameL_WBp_E);
    cls.attr("__matmul__") = cls.attr("dot");
    AddValueInstantiation<Class>(m);
    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }
  {
    using Class = SpatialMomentum<T>;
    constexpr auto& cls_doc = doc.SpatialMomentum;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialMomentum", param, cls_doc.doc);
    BindSpatialVectorMixin<T>(&cls);
    cls  // BR
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("h"), py::arg("l"), cls_doc.ctor.doc_2args)
        .def(
            py::init<const Vector6<T>&>(), py::arg("L"), cls_doc.ctor.doc_1args)
        .def("Shift", &Class::Shift, py::arg("p_BpBq_E"), cls_doc.Shift.doc)
        .def("dot", &Class::dot, py::arg("velocity"), cls_doc.dot.doc);
    constexpr char doc_dotWithArgumentNameV_IBp_E_deprecated[] =
        "The keyword argument (kwarg) has been renamed from"
        " dot(V_IBp_E) to"
        " dot(velocity)."
        " Deprecated kwarg will be unavailable after 2022-06-01.";
    cls.def("dot",
        WrapDeprecated(doc_dotWithArgumentNameV_IBp_E_deprecated, &Class::dot),
        py::arg("V_IBp_E"), doc_dotWithArgumentNameV_IBp_E_deprecated);
    cls.attr("__matmul__") = cls.attr("dot");
    AddValueInstantiation<Class>(m);
    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }
  {
    using Class = SpatialAcceleration<T>;
    constexpr auto& cls_doc = doc.SpatialAcceleration;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialAcceleration", param, cls_doc.doc);
    BindSpatialVectorMixin<T>(&cls);
    cls  // BR
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("alpha"), py::arg("a"), cls_doc.ctor.doc_2args)
        .def(py::init<const Vector6<T>&>(), py::arg("A"),
            cls_doc.ctor.doc_1args);
    cls.def("ShiftWithZeroAngularVelocity",
        &Class::ShiftWithZeroAngularVelocity, py::arg("offset"),
        cls_doc.ShiftWithZeroAngularVelocity.doc);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    constexpr char doc_ShiftOneArg_deprecatedArgName[] =
        "Shift(p_PoQ_E) is deprecated, and will be removed on or around"
        " 2022-07-01. Please instead use "
        "ShiftWithZeroAngularVelocity(offset)";
    cls.def("Shift",
        WrapDeprecated(doc_ShiftOneArg_deprecatedArgName,
            overload_cast_explicit<Class, const Vector3<T>&>(&Class::Shift)),
        py::arg("p_PoQ_E"), doc_ShiftOneArg_deprecatedArgName);
#pragma GCC diagnostic pop
    cls.def("Shift",
        overload_cast_explicit<Class, const Vector3<T>&, const Vector3<T>&>(
            &Class::Shift),
        py::arg("offset"), py::arg("angular_velocity_of_this_frame"),
        cls_doc.Shift.doc);
    constexpr char doc_ShiftTwoArg_deprecatedArgName[] =
        "The keyword arguments (kwargs) have been renamed from"
        " Shift(p_PoQ_E, w_WP_E) to"
        " Shift(offset, angular_velocity_of_this_frame)."
        " The old kwargs are deprecated and unavailable after 2022-07-01.";
    cls.def("Shift",
        WrapDeprecated(doc_ShiftTwoArg_deprecatedArgName,
            overload_cast_explicit<Class, const Vector3<T>&, const Vector3<T>&>(
                &Class::Shift)),
        py::arg("p_PoQ_E"), py::arg("w_WP_E"),
        doc_ShiftTwoArg_deprecatedArgName);
    cls.def("ComposeWithMovingFrameAcceleration",
        &Class::ComposeWithMovingFrameAcceleration,
        py::arg("position_of_moving_frame"),
        py::arg("angular_velocity_of_this_frame"),
        py::arg("velocity_of_moving_frame"),
        py::arg("acceleration_of_moving_frame"),
        cls_doc.ComposeWithMovingFrameAcceleration.doc);
    constexpr char doc_ComposeMovingFrameAccel_deprecatedArgName[] =
        "The keyword arguments (kwargs) have been renamed from"
        " ComposeWithMovingFrameAcceleration(p_PB_E, w_WP_E, V_PB_E, A_PB_E) to"
        " ComposeWithMovingFrameAcceleration(position_of_moving_frame,"
        " angular_velocity_of_this_frame,"
        " velocity_of_moving_frame,"
        " acceleration_of_moving_frame)"
        " The old kwargs are deprecated and unavailable after 2022-07-01.";
    cls.def("ComposeWithMovingFrameAcceleration",
        WrapDeprecated(doc_ComposeMovingFrameAccel_deprecatedArgName,
            &Class::ComposeWithMovingFrameAcceleration),
        py::arg("p_PB_E"), py::arg("w_WP_E"), py::arg("V_PB_E"),
        py::arg("A_PB_E"), doc_ComposeMovingFrameAccel_deprecatedArgName);
    AddValueInstantiation<Class>(m);
    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }
  {
    using Class = multibody::SpatialForce<T>;
    constexpr auto& cls_doc = doc.SpatialForce;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialForce", param, cls_doc.doc);
    BindSpatialVectorMixin<T>(&cls);
    cls  // BR
        .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                 const Eigen::Ref<const Vector3<T>>&>(),
            py::arg("tau"), py::arg("f"), cls_doc.ctor.doc_2args)
        .def(
            py::init<const Vector6<T>&>(), py::arg("F"), cls_doc.ctor.doc_1args)
        .def("Shift",
            overload_cast_explicit<Class, const Vector3<T>&>(&Class::Shift),
            py::arg("p_BpBq_E"), cls_doc.Shift.doc_1args)
        .def("dot",
            overload_cast_explicit<T, const SpatialVelocity<T>&>(&Class::dot),
            py::arg("velocity"), cls_doc.dot.doc);
    constexpr char doc_dot_deprecatedArgName[] =
        "The keyword argument (kwarg) has been renamed from"
        " dot(V_IBp_E) to"
        " dot(velocity)."
        " Deprecated kwarg will be unavailable after 2022-06-01.";
    cls.def("dot", WrapDeprecated(doc_dot_deprecatedArgName, &Class::dot),
        py::arg("V_IBp_E"), doc_dot_deprecatedArgName);
    cls.attr("__matmul__") = cls.attr("dot");
    AddValueInstantiation<Class>(m);
    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }
}
}  // namespace

PYBIND11_MODULE(math, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.math");
  py::module::import("pydrake.symbolic");

  m.doc() = "Bindings for multibody math.";
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake

#pragma GCC diagnostic pop
