#include <cmath>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
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

PYBIND11_MODULE(math, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;

  m.doc() = "Bindings for //math.";
  constexpr auto& doc = pydrake_doc.drake.math;

  auto eigen_geometry_py = py::module::import("pydrake.common.eigen_geometry");

  const char* doc_iso3_deprecation =
      "DO NOT USE!. We only offer this API for backwards compatibility with "
      "Isometry3 and it will be deprecated soon with the resolution of "
      "#9865. This will be removed on or around 2019-06-26.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
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

  {
    using Class = RigidTransform<T>;
    constexpr auto& cls_doc = doc.RigidTransform;
    py::class_<Class> cls(m, "RigidTransform", cls_doc.doc);
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
        .def("set", &Class::set, py::arg("R"), py::arg("p"), cls_doc.set.doc)
        .def("SetFromIsometry3", &Class::SetFromIsometry3, py::arg("pose"),
            cls_doc.SetFromIsometry3.doc)
        .def_static("Identity", &Class::Identity, cls_doc.Identity.doc)
        .def("rotation", &Class::rotation, py_reference_internal,
            cls_doc.rotation.doc)
        .def("set_rotation", &Class::set_rotation, py::arg("R"),
            cls_doc.set_rotation.doc)
        .def("translation", &Class::translation, py_reference_internal,
            cls_doc.translation.doc)
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
            [doc_iso3_deprecation](
                const RigidTransform<T>* self, const Isometry3<T>& other) {
              WarnDeprecated(doc_iso3_deprecation);
              return *self * RigidTransform<T>(other);
            },
            py::arg("other"), doc_iso3_deprecation)
        .def("matrix", &RigidTransform<T>::matrix, doc_iso3_deprecation)
        .def("linear", &RigidTransform<T>::linear, py_reference_internal,
            doc_iso3_deprecation);
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    // .def("IsNearlyEqualTo", ...)
    // .def("IsExactlyEqualTo", ...)
  }

  // Install constructor to Isometry3 to permit `implicitly_convertible` to
  // work.
  // TODO(eric.cousineau): Add deprecation message.
  // TODO(eric.cousineau): Consider more elegant implicit conversion process.
  // See pybind/pybind11#1735
  py::class_<Isometry3<T>>(eigen_geometry_py.attr("Isometry3"))
      .def(py::init(
          [](const RigidTransform<T>& X) { return X.GetAsIsometry3(); }));
  py::implicitly_convertible<RigidTransform<T>, Isometry3<T>>();

  {
    using Class = RollPitchYaw<T>;
    constexpr auto& cls_doc = doc.RollPitchYaw;
    py::class_<Class> cls(m, "RollPitchYaw", cls_doc.doc);
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
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = RotationMatrix<T>;
    constexpr auto& cls_doc = doc.RotationMatrix;
    py::class_<Class> cls(m, "RotationMatrix", cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const Matrix3<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<Eigen::Quaternion<T>>(), py::arg("quaternion"),
            cls_doc.ctor.doc_1args_quaternion)
        .def(py::init<const RollPitchYaw<T>&>(), py::arg("rpy"),
            cls_doc.ctor.doc_1args_rpy)
        .def("matrix", &Class::matrix, cls_doc.matrix.doc)
        .def("multiply",
            [](const Class& self, const Class& other) { return self * other; },
            cls_doc.operator_mul.doc_1args_other)
        .def("inverse", &Class::inverse, cls_doc.inverse.doc)
        .def("ToQuaternion",
            overload_cast_explicit<Eigen::Quaternion<T>>(&Class::ToQuaternion),
            cls_doc.ToQuaternion.doc_0args)
        .def_static("Identity", &Class::Identity, cls_doc.Identity.doc);
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
  }

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
  // N.B. The ordering in which the overloads are resolved will change based on
  // when modules are loaded. However, there should not be ambiguous implicit
  // conversions between autodiff and symbolic, and double overloads should
  // always occur first, so it shouldn't be a problem.
  // See`math_overloads_test`, which tests this specifically.
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

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
