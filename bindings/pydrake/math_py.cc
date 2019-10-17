#include <cmath>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/math/barycentric.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"
#include "drake/math/continuous_lyapunov_equation.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/math/discrete_lyapunov_equation.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/math/quadratic_form.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/wrap_to.h"

namespace drake {
namespace pydrake {

using symbolic::Expression;

namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  {
    // N.B. Keep the deprecation date here in sync with the deprecation comment
    // inside drake/math/rigid_transform.h.
    const char* doc_rigid_transform_linear_matrix_deprecation =
        "DO NOT USE! We offer this API for backwards compatibility with "
        "Isometry3, but it will be removed on or around 2019-12-01. "
        "See drake issue #9865 for details.";

    using Class = RigidTransform<T>;
    constexpr auto& cls_doc = doc.RigidTransform;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RigidTransform", param, cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), cls_doc.ctor.doc_2args_R_p)
        .def(py::init<const RollPitchYaw<T>&, const Vector3<T>&>(),
            py::arg("rpy"), py::arg("p"), cls_doc.ctor.doc_2args_rpy_p)
        .def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
            py::arg("quaternion"), py::arg("p"),
            cls_doc.ctor.doc_2args_quaternion_p)
        .def(py::init<const Eigen::AngleAxis<T>&, const Vector3<T>&>(),
            py::arg("theta_lambda"), py::arg("p"),
            cls_doc.ctor.doc_2args_theta_lambda_p)
        .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<const Vector3<T>&>(), py::arg("p"),
            cls_doc.ctor.doc_1args_p)
        .def(py::init<const Isometry3<T>&>(), py::arg("pose"),
            cls_doc.ctor.doc_1args_pose)
        .def(py::init<const MatrixX<T>&>(), py::arg("pose"),
            cls_doc.ctor.doc_1args_constEigenMatrixBase);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    constexpr char deprecated_matrix_ctor_doc[] =
        "Please use ``RigidTransfrom(pose=...)`` constructor. This will be "
        "removed on or around 2019-11-15.";
    cls  // BR
        .def(py_init_deprecated(deprecated_matrix_ctor_doc,
                 [](const Matrix4<T>& matrix) {
                   return Class::FromMatrix4(matrix);
                 }),
            py::arg("matrix"), deprecated_matrix_ctor_doc)
        .def_static("FromMatrix4",
            WrapDeprecated(deprecated_matrix_ctor_doc, &Class::FromMatrix4),
            py::arg("matrix"), deprecated_matrix_ctor_doc);
#pragma GCC diagnostic pop
    cls  // BR
        .def("set", &Class::set, py::arg("R"), py::arg("p"), cls_doc.set.doc)
        .def("SetFromIsometry3", &Class::SetFromIsometry3, py::arg("pose"),
            cls_doc.SetFromIsometry3.doc)
        .def_static("Identity", &Class::Identity, cls_doc.Identity.doc)
        .def("rotation", &Class::rotation, py_reference_internal,
            cls_doc.rotation.doc)
        .def("set_rotation",
            py::overload_cast<const RotationMatrix<T>&>(&Class::set_rotation),
            py::arg("R"), cls_doc.set_rotation.doc_1args_R)
        .def("set_rotation",
            py::overload_cast<const RollPitchYaw<T>&>(&Class::set_rotation),
            py::arg("rpy"), cls_doc.set_rotation.doc_1args_rpy)
        .def("set_rotation",
            py::overload_cast<const Eigen::Quaternion<T>&>(
                &Class::set_rotation),
            py::arg("quaternion"), cls_doc.set_rotation.doc_1args_quaternion)
        .def("set_rotation",
            py::overload_cast<const Eigen::AngleAxis<T>&>(&Class::set_rotation),
            py::arg("theta_lambda"),
            cls_doc.set_rotation.doc_1args_theta_lambda)
        .def("translation", &Class::translation,
            return_value_policy_for_scalar_type<T>(), cls_doc.translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("p"),
            cls_doc.set_translation.doc)
        .def("GetAsMatrix4", &Class::GetAsMatrix4, cls_doc.GetAsMatrix4.doc)
        .def("GetAsMatrix34", &Class::GetAsMatrix34, cls_doc.GetAsMatrix34.doc)
        .def("GetAsIsometry3", &Class::GetAsIsometry3,
            cls_doc.GetAsIsometry3.doc)
        .def("SetIdentity", &Class::SetIdentity, cls_doc.SetIdentity.doc)
        // .def("IsExactlyIdentity", ...)
        // .def("IsIdentityToEpsilon", ...)
        .def("inverse", &Class::inverse, cls_doc.inverse.doc)
        .def("multiply",
            [](const Class* self, const Class& other) { return *self * other; },
            py::arg("other"), cls_doc.operator_mul.doc_1args_other)
        .def("multiply",
            [](const Class* self, const Vector3<T>& p_BoQ_B) {
              return *self * p_BoQ_B;
            },
            py::arg("p_BoQ_B"), cls_doc.operator_mul.doc_1args_p_BoQ_B)
        .def("multiply",
            [](const Class* self, const Matrix3X<T>& p_BoQ_B) {
              return *self * p_BoQ_B;
            },
            py::arg("p_BoQ_B"),
            cls_doc.operator_mul.doc_1args_constEigenMatrixBase)
        .def("matrix", &RigidTransform<T>::matrix,
            doc_rigid_transform_linear_matrix_deprecation)
        .def("linear", &RigidTransform<T>::linear, py_reference_internal,
            doc_rigid_transform_linear_matrix_deprecation);
    DefPickle(&cls, [](const Class& self) { return self.GetAsMatrix34(); },
        [](const Eigen::Matrix<T, 3, 4>& matrix) { return Class(matrix); });
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, cls_doc.cast.doc);
    // .def("IsNearlyEqualTo", ...)
    // .def("IsExactlyEqualTo", ...)
  }

  {
    using Class = RotationMatrix<T>;
    constexpr auto& cls_doc = doc.RotationMatrix;

    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RotationMatrix", param, cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const Matrix3<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<Eigen::Quaternion<T>>(), py::arg("quaternion"),
            cls_doc.ctor.doc_1args_quaternion)
        .def(py::init<const Eigen::AngleAxis<T>&>(), py::arg("theta_lambda"),
            cls_doc.ctor.doc_1args_theta_lambda)
        .def(py::init<const RollPitchYaw<T>&>(), py::arg("rpy"),
            cls_doc.ctor.doc_1args_rpy)
        .def_static("MakeXRotation", &Class::MakeXRotation, py::arg("theta"),
            cls_doc.MakeXRotation.doc)
        .def_static("MakeYRotation", &Class::MakeYRotation, py::arg("theta"),
            cls_doc.MakeYRotation.doc)
        .def_static("MakeZRotation", &Class::MakeZRotation, py::arg("theta"),
            cls_doc.MakeZRotation.doc)
        .def_static("Identity", &Class::Identity, cls_doc.Identity.doc)
        .def("set", &Class::set, py::arg("R"), cls_doc.set.doc)
        .def("inverse", &Class::inverse, cls_doc.inverse.doc)
        .def("transpose", &Class::transpose, cls_doc.transpose.doc)
        .def("matrix", &Class::matrix, cls_doc.matrix.doc)
        .def("row", &Class::row, py::arg("index"), cls_doc.row.doc)
        .def("col", &Class::col, py::arg("index"), cls_doc.col.doc)
        .def("multiply",
            [](const Class& self, const Class& other) { return self * other; },
            py::arg("other"), cls_doc.operator_mul.doc_1args_other)
        .def("multiply",
            [](const Class& self, const Vector3<T>& v_B) { return self * v_B; },
            py::arg("v_B"), cls_doc.operator_mul.doc_1args_v_B)
        .def("multiply",
            [](const Class& self, const Matrix3X<T>& v_B) {
              return self * v_B;
            },
            py::arg("v_B"), cls_doc.operator_mul.doc_1args_constEigenMatrixBase)
        .def("IsValid", overload_cast_explicit<boolean<T>>(&Class::IsValid),
            cls_doc.IsValid.doc_0args)
        .def("IsExactlyIdentity", &Class::IsExactlyIdentity,
            cls_doc.IsExactlyIdentity.doc)
        .def("IsIdentityToInternalTolerance",
            &Class::IsIdentityToInternalTolerance,
            cls_doc.IsIdentityToInternalTolerance.doc)
        // Does not return the quality_factor
        .def_static("ProjectToRotationMatrix",
            [](const Matrix3<T>& M) {
              return RotationMatrix<T>::ProjectToRotationMatrix(M);
            },
            py::arg("M"), cls_doc.ProjectToRotationMatrix.doc)
        .def("ToQuaternion",
            overload_cast_explicit<Eigen::Quaternion<T>>(&Class::ToQuaternion),
            cls_doc.ToQuaternion.doc_0args);
    DefPickle(&cls, [](const Class& self) { return self.matrix(); },
        [](const Matrix3<T>& matrix) { return Class(matrix); });
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, cls_doc.cast.doc);
  }

  {
    using Class = RollPitchYaw<T>;
    constexpr auto& cls_doc = doc.RollPitchYaw;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RollPitchYaw", param, cls_doc.doc);
    cls  // BR
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const Vector3<T>>(), py::arg("rpy"),
            cls_doc.ctor.doc_1args_rpy)
        .def(py::init<const T&, const T&, const T&>(), py::arg("roll"),
            py::arg("pitch"), py::arg("yaw"),
            cls_doc.ctor.doc_3args_roll_pitch_yaw)
        .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<const Eigen::Quaternion<T>&>(), py::arg("quaternion"),
            cls_doc.ctor.doc_1args_quaternion)
        .def(py::init([](const Matrix3<T>& matrix) {
          return Class(RotationMatrix<T>(matrix));
        }),
            py::arg("matrix"),
            "Construct from raw rotation matrix. See RotationMatrix overload "
            "for more information.")
        .def("vector", &Class::vector, cls_doc.vector.doc)
        .def("roll_angle", &Class::roll_angle, cls_doc.roll_angle.doc)
        .def("pitch_angle", &Class::pitch_angle, cls_doc.pitch_angle.doc)
        .def("yaw_angle", &Class::yaw_angle, cls_doc.yaw_angle.doc)
        .def("ToQuaternion", &Class::ToQuaternion, cls_doc.ToQuaternion.doc)
        .def("ToRotationMatrix", &Class::ToRotationMatrix,
            cls_doc.ToRotationMatrix.doc)
        .def("CalcRotationMatrixDt", &Class::CalcRotationMatrixDt,
            py::arg("rpyDt"), cls_doc.CalcRotationMatrixDt.doc)
        .def("CalcAngularVelocityInParentFromRpyDt",
            &Class::CalcAngularVelocityInParentFromRpyDt, py::arg("rpyDt"),
            cls_doc.CalcAngularVelocityInParentFromRpyDt.doc)
        .def("CalcAngularVelocityInChildFromRpyDt",
            &Class::CalcAngularVelocityInChildFromRpyDt, py::arg("rpyDt"),
            cls_doc.CalcAngularVelocityInChildFromRpyDt.doc)
        .def("CalcRpyDtFromAngularVelocityInParent",
            &Class::CalcRpyDtFromAngularVelocityInParent, py::arg("w_AD_A"),
            cls_doc.CalcRpyDtFromAngularVelocityInParent.doc)
        .def("CalcRpyDDtFromRpyDtAndAngularAccelInParent",
            &Class::CalcRpyDDtFromRpyDtAndAngularAccelInParent,
            py::arg("rpyDt"), py::arg("alpha_AD_A"),
            cls_doc.CalcRpyDDtFromRpyDtAndAngularAccelInParent.doc)
        .def("CalcRpyDDtFromAngularAccelInChild",
            &Class::CalcRpyDDtFromAngularAccelInChild, py::arg("rpyDt"),
            py::arg("alpha_AD_D"),
            cls_doc.CalcRpyDDtFromAngularAccelInChild.doc);
    DefPickle(&cls, [](const Class& self) { return self.vector(); },
        [](const Vector3<T>& rpy) { return Class(rpy); });
    DefCopyAndDeepCopy(&cls);
    // N.B. `RollPitchYaw::cast` is not defined in C++.
  }

  auto eigen_geometry_py = py::module::import("pydrake.common.eigen_geometry");
  // Install constructor to Isometry3 to permit `implicitly_convertible` to
  // work.
  // TODO(eric.cousineau): Add deprecation message.
  // TODO(eric.cousineau): Consider more elegant implicit conversion process.
  // See pybind/pybind11#1735
  py::class_<Isometry3<T>>(eigen_geometry_py.attr("Isometry3"))
      .def(py::init(
          [](const RigidTransform<T>& X) { return X.GetAsIsometry3(); }));
  py::implicitly_convertible<RigidTransform<T>, Isometry3<T>>();
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  // TODO(eric.cousineau): Bind remaining classes for all available scalar
  // types.
  using T = double;
  m.def("wrap_to", &wrap_to<T, T>, py::arg("value"), py::arg("low"),
      py::arg("high"), doc.wrap_to.doc);
  m.def("ComputeBasisFromAxis",
      [](int axis_index, const Vector3<T>& axis) {
        return ComputeBasisFromAxis(axis_index, axis);
      },
      py::arg("axis_index"), py::arg("axis_W"), doc.ComputeBasisFromAxis.doc);
  py::class_<BarycentricMesh<T>>(m, "BarycentricMesh", doc.BarycentricMesh.doc)
      .def(py::init<BarycentricMesh<T>::MeshGrid>(),
          doc.BarycentricMesh.ctor.doc)
      .def("get_input_grid", &BarycentricMesh<T>::get_input_grid,
          doc.BarycentricMesh.get_input_grid.doc)
      .def("get_input_size", &BarycentricMesh<T>::get_input_size,
          doc.BarycentricMesh.get_input_size.doc)
      .def("get_num_mesh_points", &BarycentricMesh<T>::get_num_mesh_points,
          doc.BarycentricMesh.get_num_mesh_points.doc)
      .def("get_num_interpolants", &BarycentricMesh<T>::get_num_interpolants,
          doc.BarycentricMesh.get_num_interpolants.doc)
      .def("get_mesh_point",
          overload_cast_explicit<VectorX<T>, int>(
              &BarycentricMesh<T>::get_mesh_point),
          doc.BarycentricMesh.get_mesh_point.doc_1args)
      .def("get_all_mesh_points", &BarycentricMesh<T>::get_all_mesh_points,
          doc.BarycentricMesh.get_all_mesh_points.doc)
      .def("EvalBarycentricWeights",
          [](const BarycentricMesh<T>* self,
              const Eigen::Ref<const VectorX<T>>& input) {
            const int n = self->get_num_interpolants();
            Eigen::VectorXi indices(n);
            VectorX<T> weights(n);
            self->EvalBarycentricWeights(input, &indices, &weights);
            return std::make_pair(indices, weights);
          },
          doc.BarycentricMesh.EvalBarycentricWeights.doc)
      .def("Eval",
          overload_cast_explicit<VectorX<T>,
              const Eigen::Ref<const MatrixX<T>>&,
              const Eigen::Ref<const VectorX<T>>&>(&BarycentricMesh<T>::Eval),
          doc.BarycentricMesh.Eval.doc_2args)
      .def("MeshValuesFrom", &BarycentricMesh<T>::MeshValuesFrom,
          doc.BarycentricMesh.MeshValuesFrom.doc);

  // Quadratic Form.
  m  // BR
      .def("DecomposePSDmatrixIntoXtransposeTimesX",
          &DecomposePSDmatrixIntoXtransposeTimesX, py::arg("Y"),
          py::arg("zero_tol"), doc.DecomposePSDmatrixIntoXtransposeTimesX.doc)
      .def("DecomposePositiveQuadraticForm", &DecomposePositiveQuadraticForm,
          py::arg("Q"), py::arg("b"), py::arg("c"), py::arg("tol") = 0,
          doc.DecomposePositiveQuadraticForm.doc);

  // Riccati and Lyapunov Equations.
  m  // BR
      .def("ContinuousAlgebraicRiccatiEquation",
          py::overload_cast<const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&>(
              &ContinuousAlgebraicRiccatiEquation),
          py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
          doc.ContinuousAlgebraicRiccatiEquation.doc_4args_A_B_Q_R)
      .def("RealContinuousLyapunovEquation", &RealContinuousLyapunovEquation,
          py::arg("A"), py::arg("Q"), doc.RealContinuousLyapunovEquation.doc)
      .def("DiscreteAlgebraicRiccatiEquation",
          &DiscreteAlgebraicRiccatiEquation, py::arg("A"), py::arg("B"),
          py::arg("Q"), py::arg("R"), doc.DiscreteAlgebraicRiccatiEquation.doc)
      .def("RealDiscreteLyapunovEquation", &RealDiscreteLyapunovEquation,
          py::arg("A"), py::arg("Q"), doc.RealDiscreteLyapunovEquation.doc);

  // General scalar math overloads.
  // N.B. Additional overloads will be added for autodiff, symbolic, etc, by
  // those respective modules.
  // TODO(eric.cousineau): If possible, delegate these to NumPy UFuncs,
  // either using __array_ufunc__ or user dtypes.
  // TODO(m-chaturvedi) Add Pybind11 documentation.
  m  // BR
      .def("log", [](double x) { return log(x); })
      .def("abs", [](double x) { return fabs(x); })
      .def("exp", [](double x) { return exp(x); })
      .def("sqrt", [](double x) { return sqrt(x); })
      .def("pow", [](double x, double y) { return pow(x, y); })
      .def("sin", [](double x) { return sin(x); })
      .def("cos", [](double x) { return cos(x); })
      .def("tan", [](double x) { return tan(x); })
      .def("asin", [](double x) { return asin(x); })
      .def("acos", [](double x) { return acos(x); })
      .def("atan", [](double x) { return atan(x); })
      .def("atan2", [](double y, double x) { return atan2(y, x); },
          py::arg("y"), py::arg("x"))
      .def("sinh", [](double x) { return sinh(x); })
      .def("cosh", [](double x) { return cosh(x); })
      .def("tanh", [](double x) { return tanh(x); })
      .def("min", [](double x, double y) { return fmin(x, y); })
      .def("max", [](double x, double y) { return fmax(x, y); })
      .def("ceil", [](double x) { return ceil(x); })
      .def("floor", [](double x) { return floor(x); });

  // General vectorized / matrix overloads.
  m  // BR
      .def("inv", [](const Eigen::MatrixXd& X) -> Eigen::MatrixXd {
        return X.inverse();
      });

  // Add testing module.
  {
    auto mtest = m.def_submodule("_test");
    // Implicit conversions.
    mtest.def("TakeIsometry3", [](const Isometry3<T>&) { return true; });
    mtest.def(
        "TakeRigidTransform", [](const RigidTransform<T>&) { return true; });
  }

  // See TODO in corresponding header file - these should be removed soon!
  pydrake::internal::BindAutoDiffMathOverloads(&m);
  pydrake::internal::BindSymbolicMathOverloads(&m);
}
}  // namespace

PYBIND11_MODULE(math, m) {
  // N.B. Docstring contained in `_math_extra.py`.

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.symbolic");

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
  DoScalarIndependentDefinitions(m);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
