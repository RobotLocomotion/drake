#include <cmath>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/barycentric.h"
#include "drake/math/orthonormal_basis.h"
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

  py::module::import("pydrake.util.eigen_geometry");

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

  py::class_<RigidTransform<T>>(m, "RigidTransform", doc.RigidTransform.doc)
      .def(py::init(), doc.RigidTransform.ctor.doc_0args)
      .def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
          py::arg("R"), py::arg("p"), doc.RigidTransform.ctor.doc_2args_R_p)
      .def(py::init<const RollPitchYaw<T>&, const Vector3<T>&>(),
          py::arg("rpy"), py::arg("p"), doc.RigidTransform.ctor.doc_2args_rpy_p)
      .def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
          py::arg("quaternion"), py::arg("p"),
          doc.RigidTransform.ctor.doc_2args_quaternion_p)
      .def(py::init<const Eigen::AngleAxis<T>&, const Vector3<T>&>(),
          py::arg("theta_lambda"), py::arg("p"),
          doc.RigidTransform.ctor.doc_2args_theta_lambda_p)
      .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
          doc.RigidTransform.ctor.doc_1args_R)
      .def(py::init<const Vector3<T>&>(), py::arg("p"),
          doc.RigidTransform.ctor.doc_1args_p)
      .def(py::init<const Isometry3<T>&>(), py::arg("pose"),
          doc.RigidTransform.ctor.doc_1args_pose)
      .def("set", &RigidTransform<T>::set, py::arg("R"), py::arg("p"),
          doc.RigidTransform.set.doc)
      .def("SetFromIsometry3", &RigidTransform<T>::SetFromIsometry3,
          py::arg("pose"), doc.RigidTransform.SetFromIsometry3.doc)
      .def_static("Identity", &RigidTransform<T>::Identity,
          doc.RigidTransform.Identity.doc)
      .def("rotation", &RigidTransform<T>::rotation, py_reference_internal,
          doc.RigidTransform.rotation.doc)
      .def("set_rotation", &RigidTransform<T>::set_rotation, py::arg("R"),
          doc.RigidTransform.set_rotation.doc)
      .def("translation", &RigidTransform<T>::translation,
          py_reference_internal, doc.RigidTransform.translation.doc)
      .def("set_translation", &RigidTransform<T>::set_translation, py::arg("p"),
          doc.RigidTransform.set_translation.doc)
      .def("GetAsMatrix4", &RigidTransform<T>::GetAsMatrix4,
          doc.RigidTransform.GetAsMatrix4.doc)
      .def("GetAsMatrix34", &RigidTransform<T>::GetAsMatrix34,
          doc.RigidTransform.GetAsMatrix34.doc)
      .def("GetAsIsometry3", &RigidTransform<T>::GetAsIsometry3,
          doc.RigidTransform.GetAsIsometry3.doc)
      .def("SetIdentity", &RigidTransform<T>::SetIdentity,
          doc.RigidTransform.SetIdentity.doc)
      // .def("IsExactlyIdentity", ...)
      // .def("IsIdentityToEpsilon", ...)
      .def("inverse", &RigidTransform<T>::inverse,
          doc.RigidTransform.inverse.doc)
      // TODO(eric.cousineau): Use `matmul` operator once we support Python3.
      .def("multiply",
          [](const RigidTransform<T>* self, const RigidTransform<T>& other) {
            return *self * other;
          },
          py::arg("other"), doc.RigidTransform.operator_mul.doc_1args_other)
      .def("multiply",
          [](const RigidTransform<T>* self, const Vector3<T>& p_BoQ_B) {
            return *self * p_BoQ_B;
          },
          py::arg("p_BoQ_B"),
          doc.RigidTransform.operator_mul.doc_1args_p_BoQ_B);
  // .def("IsNearlyEqualTo", ...)
  // .def("IsExactlyEqualTo", ...)

  py::class_<RollPitchYaw<T>>(m, "RollPitchYaw")
      .def(py::init<const Vector3<T>>(), py::arg("rpy"),
          doc.RollPitchYaw.ctor.doc_1args_rpy)
      .def(py::init<const T&, const T&, const T&>(), py::arg("roll"),
          py::arg("pitch"), py::arg("yaw"),
          doc.RollPitchYaw.ctor.doc_3args_roll_pitch_yaw)
      .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
          doc.RollPitchYaw.ctor.doc_1args_R)
      .def(py::init<const Eigen::Quaternion<T>&>(), py::arg("quaternion"),
          doc.RollPitchYaw.ctor.doc_1args_quaternion)
      .def("vector", &RollPitchYaw<T>::vector, doc.RollPitchYaw.vector.doc)
      .def("roll_angle", &RollPitchYaw<T>::roll_angle,
          doc.RollPitchYaw.roll_angle.doc)
      .def("pitch_angle", &RollPitchYaw<T>::pitch_angle,
          doc.RollPitchYaw.pitch_angle.doc)
      .def("yaw_angle", &RollPitchYaw<T>::yaw_angle,
          doc.RollPitchYaw.yaw_angle.doc)
      .def("ToQuaternion", &RollPitchYaw<T>::ToQuaternion,
          doc.RollPitchYaw.ToQuaternion.doc)
      .def("ToRotationMatrix", &RollPitchYaw<T>::ToRotationMatrix,
          doc.RollPitchYaw.ToRotationMatrix.doc);

  py::class_<RotationMatrix<T>>(m, "RotationMatrix", doc.RotationMatrix.doc)
      .def(py::init(), doc.RotationMatrix.ctor.doc_0args)
      .def(py::init<const Matrix3<T>&>(), py::arg("R"),
          doc.RotationMatrix.ctor.doc_1args_R)
      .def(py::init<Eigen::Quaternion<T>>(), py::arg("quaternion"),
          doc.RotationMatrix.ctor.doc_1args_quaternion)
      .def(py::init<const RollPitchYaw<T>&>(), py::arg("rpy"),
          doc.RotationMatrix.ctor.doc_1args_rpy)
      .def("matrix", &RotationMatrix<T>::matrix, doc.RotationMatrix.matrix.doc)
      // Do not define an operator until we have the Python3 `@` operator so
      // that operations are similar to those of arrays.
      .def("multiply",
          [](const RotationMatrix<T>& self, const RotationMatrix<T>& other) {
            return self * other;
          },
          doc.RotationMatrix.operator_mul.doc_1args_other)
      .def("inverse", &RotationMatrix<T>::inverse,
          doc.RotationMatrix.inverse.doc)
      .def("ToQuaternion",
          overload_cast_explicit<Eigen::Quaternion<T>>(
              &RotationMatrix<T>::ToQuaternion),
          doc.RotationMatrix.ToQuaternion.doc_0args)
      .def_static("Identity", &RotationMatrix<T>::Identity,
          doc.RotationMatrix.Identity.doc);

  // General math overloads.
  // N.B. Additional overloads will be added for autodiff, symbolic, etc, by
  // those respective modules.
  // TODO(eric.cousineau): If possible, delegate these to NumPy UFuncs, either
  // using __array_ufunc__ or user dtypes.
  // N.B. The ordering in which the overloads are resolved will change based on
  // when modules are loaded. However, there should not be ambiguous implicit
  // conversions between autodiff and symbolic, and double overloads should
  // always occur first, so it shouldn't be a problem.
  // See `math_overloads_test`, which tests this specifically.
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
}

}  // namespace pydrake
}  // namespace drake
