#include "drake/math/rotation_matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::math;

namespace py = pybind11;
void apb11_pydrake_RotationMatrix_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using PyRotationMatrix_double_0 = double;

  py::class_<RotationMatrix<PyRotationMatrix_double_0>> PyRotationMatrix_double(
      m, "RotationMatrix_double");

  PyRotationMatrix_double
      .def(py::init<RotationMatrix<PyRotationMatrix_double_0> const &>(),
           py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<
               Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("R"))
      .def(py::init<::Eigen::Quaternion<double, 0> const &>(),
           py::arg("quaternion"))
      .def(py::init<::Eigen::AngleAxis<PyRotationMatrix_double_0> const &>(),
           py::arg("theta_lambda"))
      .def(py::init<RollPitchYaw<PyRotationMatrix_double_0> const &>(),
           py::arg("rpy"))
      .def("GetMaximumAbsoluteDifference",
           static_cast<double (RotationMatrix<PyRotationMatrix_double_0>::*)(
               RotationMatrix<PyRotationMatrix_double_0> const &) const>(
               &RotationMatrix<
                   PyRotationMatrix_double_0>::GetMaximumAbsoluteDifference),
           py::arg("other"))
      .def("GetMeasureOfOrthonormality",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.GetMeasureOfOrthonormality(R);
           })
      .def_static(
          "Identity",
          static_cast<RotationMatrix<PyRotationMatrix_double_0> const &(*)()>(
              &RotationMatrix<PyRotationMatrix_double_0>::Identity))
      .def("IsExactlyEqualTo",
           static_cast<
               ::drake::scalar_predicate<PyRotationMatrix_double_0>::type (
                   RotationMatrix<PyRotationMatrix_double_0>::*)(
                   RotationMatrix<PyRotationMatrix_double_0> const &) const>(
               &RotationMatrix<PyRotationMatrix_double_0>::IsExactlyEqualTo),
           py::arg("other"))
      .def("IsExactlyIdentity",
           static_cast<
               ::drake::scalar_predicate<PyRotationMatrix_double_0>::type (
                   RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::IsExactlyIdentity))
      .def("IsIdentityToInternalTolerance",
           static_cast<
               ::drake::scalar_predicate<PyRotationMatrix_double_0>::type (
                   RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<
                   PyRotationMatrix_double_0>::IsIdentityToInternalTolerance))
      .def("IsNearlyEqualTo",
           static_cast<
               ::drake::scalar_predicate<PyRotationMatrix_double_0>::type (
                   RotationMatrix<PyRotationMatrix_double_0>::*)(
                   RotationMatrix<PyRotationMatrix_double_0> const &, double)
                   const>(
               &RotationMatrix<PyRotationMatrix_double_0>::IsNearlyEqualTo),
           py::arg("other"), py::arg("tolerance"))
      .def("IsOrthonormal",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsOrthonormal(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsValid(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.IsValid(R);
           })
      .def("IsValid",
           static_cast<
               ::drake::scalar_predicate<PyRotationMatrix_double_0>::type (
                   RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::IsValid))
      .def("MakeFromOrthonormalColumns",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bx,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &By,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bz) {
             return self.MakeFromOrthonormalColumns(Bx, By, Bz);
           })
      .def("MakeFromOrthonormalRows",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ax,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ay,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Az) {
             return self.MakeFromOrthonormalRows(Ax, Ay, Az);
           })
      .def_static(
          "MakeXRotation",
          static_cast<RotationMatrix<PyRotationMatrix_double_0> (*)(
              double const &)>(
              &RotationMatrix<PyRotationMatrix_double_0>::MakeXRotation),
          py::arg("theta"))
      .def_static(
          "MakeYRotation",
          static_cast<RotationMatrix<PyRotationMatrix_double_0> (*)(
              double const &)>(
              &RotationMatrix<PyRotationMatrix_double_0>::MakeYRotation),
          py::arg("theta"))
      .def_static(
          "MakeZRotation",
          static_cast<RotationMatrix<PyRotationMatrix_double_0> (*)(
              double const &)>(
              &RotationMatrix<PyRotationMatrix_double_0>::MakeZRotation),
          py::arg("theta"))
      .def("ProjectToRotationMatrix",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M,
              double *quality_factor) {
             return self.ProjectToRotationMatrix(M, quality_factor);
           })
      .def("ToAngleAxis",
           static_cast<::Eigen::AngleAxis<PyRotationMatrix_double_0> (
               RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::ToAngleAxis))
      .def("ToQuaternion",
           static_cast<::Eigen::Quaternion<double, 0> (
               RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::ToQuaternion))
      .def("ToQuaternion",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 0,
                           Eigen::OuterStride<-1>> const &M) {
             return self.ToQuaternion(M);
           })
      .def("ToQuaternionAsVector4",
           static_cast<::Eigen::Matrix<double, 4, 1, 0, 4, 1> (
               RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<
                   PyRotationMatrix_double_0>::ToQuaternionAsVector4))
      .def("ToQuaternionAsVector4",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M) {
             return self.ToQuaternionAsVector4(M);
           })
      .def("col",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 3, 1,
               true> const (RotationMatrix<PyRotationMatrix_double_0>::*)(int)
                           const>(
               &RotationMatrix<PyRotationMatrix_double_0>::col),
           py::arg("index"))
      .def_static("get_internal_tolerance_for_orthonormality",
                  static_cast<double (*)()>(
                      &RotationMatrix<PyRotationMatrix_double_0>::
                          get_internal_tolerance_for_orthonormality))
      .def("inverse",
           static_cast<RotationMatrix<PyRotationMatrix_double_0> (
               RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::inverse))
      .def("matrix",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &(
               RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::matrix),
           py::return_value_policy::reference_internal)
      .def("row",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 1, 3,
               false> const (RotationMatrix<PyRotationMatrix_double_0>::*)(int)
                           const>(
               &RotationMatrix<PyRotationMatrix_double_0>::row),
           py::arg("index"))
      .def("set",
           [](RotationMatrix<PyRotationMatrix_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.set(R);
           })
      .def("transpose",
           static_cast<RotationMatrix<PyRotationMatrix_double_0> (
               RotationMatrix<PyRotationMatrix_double_0>::*)() const>(
               &RotationMatrix<PyRotationMatrix_double_0>::transpose))

      .def("__mul__",
           static_cast<RotationMatrix<PyRotationMatrix_double_0> (
               RotationMatrix<PyRotationMatrix_double_0>::*)(
               RotationMatrix<PyRotationMatrix_double_0> const &) const>(
               &RotationMatrix<PyRotationMatrix_double_0>::operator*),
           py::arg("other"))
      .def("__mul__",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RotationMatrix<PyRotationMatrix_double_0>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RotationMatrix<PyRotationMatrix_double_0>::operator*),
           py::arg("v_B"))
      .def("__imul__",
           static_cast<RotationMatrix<PyRotationMatrix_double_0> &(
               RotationMatrix<PyRotationMatrix_double_0>::
                   *)(RotationMatrix<PyRotationMatrix_double_0> const &)>(
               &RotationMatrix<PyRotationMatrix_double_0>::operator*=),
           py::arg("other"));

  using PyRotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd_0 =
      drake::AutoDiffXd;

  py::class_<RotationMatrix<drake::AutoDiffXd>>
      PyRotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "RotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyRotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def(py::init<
               Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("arg0"))
      .def(py::init<>())
      .def(
          py::init<
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
          py::arg("R"))
      .def(py::init<::Eigen::Quaternion<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        0> const &>(),
           py::arg("quaternion"))
      .def(py::init<::Eigen::AngleAxis<drake::AutoDiffXd> const &>(),
           py::arg("theta_lambda"))
      .def(py::init<RollPitchYaw<drake::AutoDiffXd> const &>(), py::arg("rpy"))
      .def(
          "GetMaximumAbsoluteDifference",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &other) {
            return self.GetMaximumAbsoluteDifference(other);
          })
      .def("GetMeasureOfOrthonormality",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.GetMeasureOfOrthonormality(R);
           })
      .def_static("Identity",
                  static_cast<RotationMatrix<drake::AutoDiffXd> const &(*)()>(
                      &RotationMatrix<drake::AutoDiffXd>::Identity),
                  py::return_value_policy::reference_internal)
      .def(
          "IsExactlyEqualTo",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &other) {
            return self.IsExactlyEqualTo(other);
          })
      .def("IsExactlyIdentity",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::IsExactlyIdentity))
      .def("IsIdentityToInternalTolerance",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<
                   drake::AutoDiffXd>::IsIdentityToInternalTolerance))
      .def("IsNearlyEqualTo",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &other,
              double tolerance) {
             return self.IsNearlyEqualTo(other, tolerance);
           })
      .def("IsOrthonormal",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsOrthonormal(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsValid(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.IsValid(R);
           })
      .def("IsValid",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::IsValid))
      .def(
          "MakeFromOrthonormalColumns",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bx,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &By,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bz) {
            return self.MakeFromOrthonormalColumns(Bx, By, Bz);
          })
      .def(
          "MakeFromOrthonormalRows",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ax,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ay,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Az) {
            return self.MakeFromOrthonormalRows(Ax, Ay, Az);
          })
      .def(
          "MakeXRotation",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &theta) {
            return self.MakeXRotation(theta);
          })
      .def(
          "MakeYRotation",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &theta) {
            return self.MakeYRotation(theta);
          })
      .def(
          "MakeZRotation",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &theta) {
            return self.MakeZRotation(theta);
          })
      .def("ProjectToRotationMatrix",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M,
              Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> *, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  quality_factor) {
             return self.ProjectToRotationMatrix(M, quality_factor);
           })
      .def("ToAngleAxis", static_cast<::Eigen::AngleAxis<drake::AutoDiffXd> (
                              RotationMatrix<drake::AutoDiffXd>::*)() const>(
                              &RotationMatrix<drake::AutoDiffXd>::ToAngleAxis))
      .def("ToQuaternion",
           static_cast<
               ::Eigen::Quaternion<Eigen::AutoDiffScalar<Eigen::VectorXd>, 0> (
                   RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::ToQuaternion))
      .def("ToQuaternion",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              ::Eigen::Ref<
                  const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, 3,
                                      3, 0, 3, 3>,
                  0, Eigen::OuterStride<-1>> const &M) {
             return self.ToQuaternion(M);
           })
      .def("ToQuaternionAsVector4",
           static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                       4, 1, 0, 4, 1> (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::ToQuaternionAsVector4))
      .def("ToQuaternionAsVector4",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M) {
             return self.ToQuaternionAsVector4(M);
           })
      .def("col",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, 3, 3,
                                   0, 3, 3>,
               3, 1, true> const (RotationMatrix<drake::AutoDiffXd>::*)(int)
                           const>(&RotationMatrix<drake::AutoDiffXd>::col),
           py::arg("index"))
      .def_static("get_internal_tolerance_for_orthonormality",
                  static_cast<double (*)()>(
                      &RotationMatrix<drake::AutoDiffXd>::
                          get_internal_tolerance_for_orthonormality))
      .def("inverse", static_cast<RotationMatrix<drake::AutoDiffXd> (
                          RotationMatrix<drake::AutoDiffXd>::*)() const>(
                          &RotationMatrix<drake::AutoDiffXd>::inverse))
      .def("matrix",
           static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                       3, 3, 0, 3, 3> const
                           &(RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::matrix),
           py::return_value_policy::reference_internal)
      .def("row",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, 3, 3,
                                   0, 3, 3>,
               1, 3, false> const (RotationMatrix<drake::AutoDiffXd>::*)(int)
                           const>(&RotationMatrix<drake::AutoDiffXd>::row),
           py::arg("index"))
      .def("set",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 3, 0, 3, 3> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.set(R);
           })
      .def("transpose", static_cast<RotationMatrix<drake::AutoDiffXd> (
                            RotationMatrix<drake::AutoDiffXd>::*)() const>(
                            &RotationMatrix<drake::AutoDiffXd>::transpose))

      .def("__mul__",
           static_cast<RotationMatrix<drake::AutoDiffXd> (
               RotationMatrix<drake::AutoDiffXd>::*)(
               RotationMatrix<drake::AutoDiffXd> const &) const>(
               &RotationMatrix<drake::AutoDiffXd>::operator*),
           py::arg("other"))
      .def("__mul__",
           static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                       3, 1, 0, 3, 1> (
               RotationMatrix<drake::AutoDiffXd>::*)(
               ::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, 3, 1, 0,
                               3, 1> const &) const>(
               &RotationMatrix<drake::AutoDiffXd>::operator*),
           py::arg("v_B"))
      .def("__imul__",
           static_cast<RotationMatrix<drake::AutoDiffXd> &(
               RotationMatrix<drake::AutoDiffXd>::
                   *)(RotationMatrix<drake::AutoDiffXd> const &)>(
               &RotationMatrix<drake::AutoDiffXd>::operator*=),
           py::arg("other"));
}
