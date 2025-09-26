#include "pybind11/eval.h"

#include "drake/bindings/generated_docstrings/multibody_tree.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/multibody/tree_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace pydrake {
namespace internal {

using multibody::SpatialAcceleration;
using multibody::SpatialVelocity;

namespace {

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc_multibody_tree.drake.multibody;

  {
    m.def("CalcSpatialInertia",
        py::overload_cast<const geometry::Shape&, double>(&CalcSpatialInertia),
        py::arg("shape"), py::arg("density"), doc.CalcSpatialInertia.doc_shape);

    m.def("CalcSpatialInertia",
        py::overload_cast<const geometry::TriangleSurfaceMesh<double>&, double>(
            &CalcSpatialInertia),
        py::arg("mesh"), py::arg("density"), doc.CalcSpatialInertia.doc_mesh);
  }
}

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc_multibody_tree.drake.multibody;

  {
    using Class = RotationalInertia<T>;
    constexpr auto& cls_doc = doc.RotationalInertia;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RotationalInertia", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const T&, const T&, const T&>(), py::arg("Ixx"),
            py::arg("Iyy"), py::arg("Izz"), cls_doc.ctor.doc_3args)
        .def(py::init<const T&, const T&, const T&, const T&, const T&,
                 const T&>(),
            py::arg("Ixx"), py::arg("Iyy"), py::arg("Izz"), py::arg("Ixy"),
            py::arg("Ixz"), py::arg("Iyz"), cls_doc.ctor.doc_6args)
        .def(py::init<const T&, const Vector3<T>&>(), py::arg("mass"),
            py::arg("p_PQ_E"), cls_doc.ctor.doc_2args)
        .def_static("TriaxiallySymmetric", &Class::TriaxiallySymmetric,
            py::arg("I_triaxial"), cls_doc.TriaxiallySymmetric.doc)
        .def("rows", &Class::rows, cls_doc.rows.doc)
        .def("cols", &Class::cols, cls_doc.cols.doc)
        .def("get_moments", &Class::get_moments, cls_doc.get_moments.doc)
        .def("get_products", &Class::get_products, cls_doc.get_products.doc)
        .def("Trace", &Class::Trace, cls_doc.Trace.doc)
        .def("CalcMaximumPossibleMomentOfInertia",
            &Class::CalcMaximumPossibleMomentOfInertia,
            cls_doc.CalcMaximumPossibleMomentOfInertia.doc)
        .def(
            "__getitem__",
            [](const Class& self, py::tuple key) -> T {
              if (key.size() != 2) {
                throw std::out_of_range("Expected [i,j] for __getitem__.");
              }
              const int i = py::cast<int>(key[0]);
              const int j = py::cast<int>(key[1]);
              return self(i, j);
            },
            cls_doc.operator_call.doc)
        .def("CopyToFullMatrix3", &Class::CopyToFullMatrix3,
            cls_doc.CopyToFullMatrix3.doc)
        .def("IsNearlyEqualTo", &Class::IsNearlyEqualTo, py::arg("other"),
            py::arg("precision"), cls_doc.IsNearlyEqualTo.doc)
        .def(py::self += py::self, cls_doc.operator_iadd.doc)
        .def(py::self + py::self, cls_doc.operator_add.doc)
        .def(py::self -= py::self, cls_doc.operator_isub.doc)
        .def(py::self - py::self, cls_doc.operator_sub.doc)
        .def(py::self *= T{}, cls_doc.operator_imul.doc)
        .def(py::self * T{}, cls_doc.operator_mul.doc)
        .def(T{} * py::self, cls_doc.operator_mul.doc)
        .def(py::self * Vector3<T>{}, cls_doc.operator_mul.doc)
        .def(py::self /= T{}, cls_doc.operator_idiv.doc)
        .def(py::self / T{}, cls_doc.operator_div.doc)
        .def("SetToNaN", &Class::SetToNaN, cls_doc.SetToNaN.doc)
        .def("SetZero", &Class::SetZero, cls_doc.SetZero.doc)
        .def("IsNaN", &Class::IsNaN, cls_doc.IsNaN.doc)
        .def("IsZero", &Class::IsZero, cls_doc.IsZero.doc)
        // TODO(jwnimmer-tri) Need to bind cast<>.
        .def("CalcPrincipalMomentsOfInertia",
            &Class::CalcPrincipalMomentsOfInertia,
            cls_doc.CalcPrincipalMomentsOfInertia.doc)
        .def("CalcPrincipalMomentsAndAxesOfInertia",
            &Class::CalcPrincipalMomentsAndAxesOfInertia,
            cls_doc.CalcPrincipalMomentsAndAxesOfInertia.doc)
        .def("CouldBePhysicallyValid", &Class::CouldBePhysicallyValid,
            cls_doc.CouldBePhysicallyValid.doc)
        .def("ReExpress", &Class::ReExpress, py::arg("R_AE"),
            cls_doc.ReExpress.doc)
        .def("ShiftFromCenterOfMass", &Class::ShiftFromCenterOfMass,
            py::arg("mass"), py::arg("p_BcmQ_E"),
            cls_doc.ShiftFromCenterOfMass.doc)
        .def("ShiftToCenterOfMass", &Class::ShiftToCenterOfMass,
            py::arg("mass"), py::arg("p_QBcm_E"),
            cls_doc.ShiftToCenterOfMass.doc)
        .def("ShiftToThenAwayFromCenterOfMass",
            &Class::ShiftToThenAwayFromCenterOfMass, py::arg("mass"),
            py::arg("p_PBcm_E"), py::arg("p_QBcm_E"),
            cls_doc.ShiftToThenAwayFromCenterOfMass.doc)
        .def(py::pickle(
            [](const Class& self) { return self.CopyToFullMatrix3(); },
            [](const Matrix3<T>& I) {
              // Invoke 6-argument constructor by specifying full (upper
              // diagonal) inertia matrix.
              return Class(
                  I(0, 0), I(1, 1), I(2, 2), I(0, 1), I(0, 2), I(1, 2));
            }));
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = UnitInertia<T>;
    constexpr auto& cls_doc = doc.UnitInertia;
    auto cls = DefineTemplateClassWithDefault<Class, RotationalInertia<T>>(
        m, "UnitInertia", param, cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const T&, const T&, const T&>(), py::arg("Ixx"),
            py::arg("Iyy"), py::arg("Izz"), cls_doc.ctor.doc_3args)
        .def(py::init<const T&, const T&, const T&, const T&, const T&,
                 const T&>(),
            py::arg("Ixx"), py::arg("Iyy"), py::arg("Izz"), py::arg("Ixy"),
            py::arg("Ixz"), py::arg("Iyz"), cls_doc.ctor.doc_6args)
        .def(py::init([](const RotationalInertia<T>& I) { return Class(I); }),
            py::arg("I"), cls_doc.ctor.doc_1args)
        .def("SetFromRotationalInertia", &Class::SetFromRotationalInertia,
            py::arg("I"), py::arg("mass"), py_rvp::reference,
            cls_doc.SetFromRotationalInertia.doc)
        .def("ReExpress", &Class::ReExpress, py::arg("R_AE"),
            cls_doc.ReExpress.doc)
        .def("ShiftFromCenterOfMass", &Class::ShiftFromCenterOfMass,
            py::arg("p_BcmQ_E"), cls_doc.ShiftFromCenterOfMass.doc)
        .def("ShiftToCenterOfMass", &Class::ShiftToCenterOfMass,
            py::arg("p_QBcm_E"), cls_doc.ShiftToCenterOfMass.doc)
        .def_static("PointMass", &Class::PointMass, py::arg("p_FQ"),
            cls_doc.PointMass.doc)
        .def_static("SolidEllipsoid", &Class::SolidEllipsoid, py::arg("a"),
            py::arg("b"), py::arg("c"), cls_doc.SolidEllipsoid.doc)
        .def_static("SolidSphere", &Class::SolidSphere, py::arg("r"),
            cls_doc.SolidSphere.doc)
        .def_static("HollowSphere", &Class::HollowSphere, py::arg("r"),
            cls_doc.HollowSphere.doc)
        .def_static("SolidBox", &Class::SolidBox, py::arg("Lx"), py::arg("Ly"),
            py::arg("Lz"), cls_doc.SolidBox.doc)
        .def_static(
            "SolidCube", &Class::SolidCube, py::arg("L"), cls_doc.SolidCube.doc)
        .def_static("SolidCylinder", &Class::SolidCylinder, py::arg("radius"),
            py::arg("length"), py::arg("unit_vector"),
            cls_doc.SolidCylinder.doc)
        .def_static("SolidCapsule", &Class::SolidCapsule, py::arg("radius"),
            py::arg("length"), py::arg("unit_vector"), cls_doc.SolidCapsule.doc)
        .def_static("SolidCylinderAboutEnd", &Class::SolidCylinderAboutEnd,
            py::arg("radius"), py::arg("length"), py::arg("unit_vector"),
            cls_doc.SolidCylinderAboutEnd.doc)
        .def_static("AxiallySymmetric", &Class::AxiallySymmetric,
            py::arg("moment_parallel"), py::arg("moment_perpendicular"),
            py::arg("unit_vector"), cls_doc.AxiallySymmetric.doc)
        .def_static("StraightLine", &Class::StraightLine,
            py::arg("moment_perpendicular"), py::arg("unit_vector"),
            cls_doc.StraightLine.doc)
        .def_static("ThinRod", &Class::ThinRod, py::arg("length"),
            py::arg("unit_vector"), cls_doc.ThinRod.doc)
        .def_static("TriaxiallySymmetric", &Class::TriaxiallySymmetric,
            py::arg("I_triaxial"), cls_doc.TriaxiallySymmetric.doc)
        .def(py::pickle(
            [](const Class& self) { return self.CopyToFullMatrix3(); },
            [](const Matrix3<T>& I) {
              // Invoke 6-argument constructor by specifying full (upper
              // diagonal) inertia matrix.
              return Class(
                  I(0, 0), I(1, 1), I(2, 2), I(0, 1), I(0, 2), I(1, 2));
            }));
    DefCopyAndDeepCopy(&cls);
  }

  // SpatialInertia
  {
    using Class = SpatialInertia<T>;
    constexpr auto& cls_doc = doc.SpatialInertia;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SpatialInertia", param, cls_doc.doc);
    cls  // BR
        .def_static("MakeFromCentralInertia", &Class::MakeFromCentralInertia,
            py::arg("mass"), py::arg("p_PScm_E"), py::arg("I_SScm_E"),
            cls_doc.MakeFromCentralInertia.doc)
        .def_static("SolidBoxWithDensity", &Class::SolidBoxWithDensity,
            py::arg("density"), py::arg("lx"), py::arg("ly"), py::arg("lz"),
            cls_doc.SolidBoxWithDensity.doc)
        .def_static("SolidBoxWithMass", &Class::SolidBoxWithMass,
            py::arg("mass"), py::arg("lx"), py::arg("ly"), py::arg("lz"),
            cls_doc.SolidBoxWithMass.doc)
        .def_static("SolidCubeWithDensity", &Class::SolidCubeWithDensity,
            py::arg("density"), py::arg("length"),
            cls_doc.SolidCubeWithDensity.doc)
        .def_static("SolidCapsuleWithDensity", &Class::SolidCapsuleWithDensity,
            py::arg("density"), py::arg("radius"), py::arg("length"),
            py::arg("unit_vector"), cls_doc.SolidCapsuleWithDensity.doc)
        .def_static("SolidCapsuleWithMass", &Class::SolidCapsuleWithMass,
            py::arg("mass"), py::arg("radius"), py::arg("length"),
            py::arg("unit_vector"), cls_doc.SolidCapsuleWithMass.doc)
        .def_static("SolidCylinderWithDensity",
            &Class::SolidCylinderWithDensity, py::arg("density"),
            py::arg("radius"), py::arg("length"), py::arg("unit_vector"),
            cls_doc.SolidCylinderWithDensity.doc)
        .def_static("SolidCylinderWithMass", &Class::SolidCylinderWithMass,
            py::arg("mass"), py::arg("radius"), py::arg("length"),
            py::arg("unit_vector"), cls_doc.SolidCylinderWithMass.doc)
        .def_static("SolidCylinderWithDensityAboutEnd",
            &Class::SolidCylinderWithDensityAboutEnd, py::arg("density"),
            py::arg("radius"), py::arg("length"), py::arg("unit_vector"),
            cls_doc.SolidCylinderWithDensityAboutEnd.doc)
        .def_static("SolidCylinderWithMassAboutEnd",
            &Class::SolidCylinderWithMassAboutEnd, py::arg("mass"),
            py::arg("radius"), py::arg("length"), py::arg("unit_vector"),
            cls_doc.SolidCylinderWithMassAboutEnd.doc)
        .def_static("ThinRodWithMass", &Class::ThinRodWithMass, py::arg("mass"),
            py::arg("length"), py::arg("unit_vector"),
            cls_doc.ThinRodWithMass.doc)
        .def_static("ThinRodWithMassAboutEnd", &Class::ThinRodWithMassAboutEnd,
            py::arg("mass"), py::arg("length"), py::arg("unit_vector"),
            cls_doc.ThinRodWithMassAboutEnd.doc)
        .def_static("SolidEllipsoidWithDensity",
            &Class::SolidEllipsoidWithDensity, py::arg("density"), py::arg("a"),
            py::arg("b"), py::arg("c"), cls_doc.SolidEllipsoidWithDensity.doc)
        .def_static("SolidEllipsoidWithMass", &Class::SolidEllipsoidWithMass,
            py::arg("mass"), py::arg("a"), py::arg("b"), py::arg("c"),
            cls_doc.SolidEllipsoidWithMass.doc)
        .def_static("SolidSphereWithDensity", &Class::SolidSphereWithDensity,
            py::arg("density"), py::arg("radius"),
            cls_doc.SolidSphereWithDensity.doc)
        .def_static("SolidSphereWithMass", &Class::SolidSphereWithMass,
            py::arg("mass"), py::arg("radius"), cls_doc.SolidSphereWithMass.doc)
        .def_static("HollowSphereWithDensity", &Class::HollowSphereWithDensity,
            py::arg("area_density"), py::arg("radius"),
            cls_doc.HollowSphereWithDensity.doc)
        .def_static("HollowSphereWithMass", &Class::HollowSphereWithMass,
            py::arg("mass"), py::arg("radius"),
            cls_doc.HollowSphereWithMass.doc)
        .def_static("Zero", &Class::Zero, cls_doc.Zero.doc)
        .def_static("NaN", &Class::NaN, cls_doc.NaN.doc)
        .def(py::init<const T&, const Eigen::Ref<const Vector3<T>>&,
                 const UnitInertia<T>&, const bool>(),
            py::arg("mass"), py::arg("p_PScm_E"), py::arg("G_SP_E"),
            py::arg("skip_validity_check") = false, cls_doc.ctor.doc)
        // TODO(jwnimmer-tri) Need to bind cast<>.
        .def("get_mass", &Class::get_mass, cls_doc.get_mass.doc)
        .def("get_com", &Class::get_com, cls_doc.get_com.doc)
        .def("CalcComMoment", &Class::CalcComMoment, cls_doc.CalcComMoment.doc)
        .def("get_unit_inertia", &Class::get_unit_inertia,
            cls_doc.get_unit_inertia.doc)
        .def("CalcRotationalInertia", &Class::CalcRotationalInertia,
            cls_doc.CalcRotationalInertia.doc)
        .def("IsPhysicallyValid", &Class::IsPhysicallyValid,
            cls_doc.IsPhysicallyValid.doc)
        .def("CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid",
            &Class::CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid,
            cls_doc.CalcPrincipalSemiDiametersAndPoseForSolidEllipsoid.doc)
        .def("CopyToFullMatrix6", &Class::CopyToFullMatrix6,
            cls_doc.CopyToFullMatrix6.doc)
        .def("IsNaN", &Class::IsNaN, cls_doc.IsNaN.doc)
        .def("IsZero", &Class::IsZero, cls_doc.IsZero.doc)
        .def("SetNaN", &Class::SetNaN, cls_doc.SetNaN.doc)
        .def("ReExpress", &Class::ReExpress, py::arg("R_AE"),
            cls_doc.ReExpress.doc)
        .def("Shift", &Class::Shift, py::arg("p_PQ_E"), cls_doc.Shift.doc)
        .def(py::self += py::self)
        .def(py::self * SpatialAcceleration<T>())
        .def(py::self * SpatialVelocity<T>())
        .def("__repr__",
            [](const Class& self) -> py::object {
              if constexpr (std::is_same_v<T, double>) {
                if (self.IsZero()) {
                  return py::str("SpatialInertia.Zero()");
                }
              }
              return py::eval("object.__repr__")(self);
            })
        .def(py::pickle(
            [](const Class& self) {
              return py::make_tuple(
                  self.get_mass(), self.get_com(), self.get_unit_inertia());
            },
            [](py::tuple t) {
              DRAKE_THROW_UNLESS(t.size() == 3);
              return Class(t[0].cast<T>(), t[1].cast<Vector3<T>>(),
                  t[2].cast<UnitInertia<T>>());
            }));
    DefCopyAndDeepCopy(&cls);
  }
}

}  // namespace

void DefineTreeInertia(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
