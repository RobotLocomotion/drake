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
  py::class_<RotationMatrix<double>> PyRotationMatrix_double(
      m, "RotationMatrix_double");

  PyRotationMatrix_double
      .def(py::init<RotationMatrix<double> const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<
               Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("R"))
      .def(py::init<::Eigen::Quaternion<double, 0> const &>(),
           py::arg("quaternion"))
      .def(py::init<::Eigen::AngleAxis<double> const &>(),
           py::arg("theta_lambda"))
      .def(py::init<RollPitchYaw<double> const &>(), py::arg("rpy"))
      .def("GetMaximumAbsoluteDifference",
           static_cast<double (RotationMatrix<double>::*)(
               RotationMatrix<double> const &) const>(
               &RotationMatrix<double>::GetMaximumAbsoluteDifference),
           py::arg("other"))
      .def("GetMeasureOfOrthonormality",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.GetMeasureOfOrthonormality(R);
           })
      .def_static("Identity", static_cast<RotationMatrix<double> const &(*)()>(
                                  &RotationMatrix<double>::Identity))
      .def(
          "IsExactlyEqualTo",
          static_cast<::drake::scalar_predicate<double>::type (
              RotationMatrix<double>::*)(RotationMatrix<double> const &) const>(
              &RotationMatrix<double>::IsExactlyEqualTo),
          py::arg("other"))
      .def("IsExactlyIdentity",
           static_cast<::drake::scalar_predicate<double>::type (
               RotationMatrix<double>::*)() const>(
               &RotationMatrix<double>::IsExactlyIdentity))
      .def("IsIdentityToInternalTolerance",
           static_cast<::drake::scalar_predicate<double>::type (
               RotationMatrix<double>::*)() const>(
               &RotationMatrix<double>::IsIdentityToInternalTolerance))
      .def(
          "IsNearlyEqualTo",
          static_cast<::drake::scalar_predicate<double>::type (
              RotationMatrix<double>::*)(RotationMatrix<double> const &, double)
                          const>(&RotationMatrix<double>::IsNearlyEqualTo),
          py::arg("other"), py::arg("tolerance"))
      .def("IsOrthonormal",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsOrthonormal(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsValid(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.IsValid(R);
           })
      .def("IsValid", static_cast<::drake::scalar_predicate<double>::type (
                          RotationMatrix<double>::*)() const>(
                          &RotationMatrix<double>::IsValid))
      .def("MakeFromOrthonormalColumns",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bx,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &By,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bz) {
             return self.MakeFromOrthonormalColumns(Bx, By, Bz);
           })
      .def("MakeFromOrthonormalRows",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ax,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ay,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Az) {
             return self.MakeFromOrthonormalRows(Ax, Ay, Az);
           })
      .def_static("MakeXRotation",
                  static_cast<RotationMatrix<double> (*)(double const &)>(
                      &RotationMatrix<double>::MakeXRotation),
                  py::arg("theta"))
      .def_static("MakeYRotation",
                  static_cast<RotationMatrix<double> (*)(double const &)>(
                      &RotationMatrix<double>::MakeYRotation),
                  py::arg("theta"))
      .def_static("MakeZRotation",
                  static_cast<RotationMatrix<double> (*)(double const &)>(
                      &RotationMatrix<double>::MakeZRotation),
                  py::arg("theta"))
      .def("ProjectToRotationMatrix",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M,
              double *quality_factor) {
             return self.ProjectToRotationMatrix(M, quality_factor);
           })
      .def("ToAngleAxis",
           static_cast<::Eigen::AngleAxis<double> (RotationMatrix<double>::*)()
                           const>(&RotationMatrix<double>::ToAngleAxis))
      .def("ToQuaternion", static_cast<::Eigen::Quaternion<double, 0> (
                               RotationMatrix<double>::*)() const>(
                               &RotationMatrix<double>::ToQuaternion))
      .def("ToQuaternion",
           [](RotationMatrix<double> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 0,
                           Eigen::OuterStride<-1>> const &M) {
             return self.ToQuaternion(M);
           })
      .def("ToQuaternionAsVector4",
           static_cast<::Eigen::Matrix<double, 4, 1, 0, 4, 1> (
               RotationMatrix<double>::*)() const>(
               &RotationMatrix<double>::ToQuaternionAsVector4))
      .def("ToQuaternionAsVector4",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M) {
             return self.ToQuaternionAsVector4(M);
           })
      .def("col",
           static_cast<
               ::Eigen::Block<const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 3, 1,
                              true> const (RotationMatrix<double>::*)(int)
                   const>(&RotationMatrix<double>::col),
           py::arg("index"))
      .def_static("get_internal_tolerance_for_orthonormality",
                  static_cast<double (*)()>(
                      &RotationMatrix<
                          double>::get_internal_tolerance_for_orthonormality))
      .def("inverse",
           static_cast<RotationMatrix<double> (RotationMatrix<double>::*)()
                           const>(&RotationMatrix<double>::inverse))
      .def("matrix",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &(
               RotationMatrix<double>::*)() const>(
               &RotationMatrix<double>::matrix),
           py::return_value_policy::reference_internal)
      .def("row",
           static_cast<
               ::Eigen::Block<const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 1, 3,
                              false> const (RotationMatrix<double>::*)(int)
                   const>(&RotationMatrix<double>::row),
           py::arg("index"))
      .def("set",
           [](RotationMatrix<double> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.set(R);
           })
      .def("transpose",
           static_cast<RotationMatrix<double> (RotationMatrix<double>::*)()
                           const>(&RotationMatrix<double>::transpose))

      .def("__mul__",
           static_cast<RotationMatrix<double> (RotationMatrix<double>::*)(
               RotationMatrix<double> const &) const>(
               &RotationMatrix<double>::operator*),
           py::arg("other"))
      .def("__mul__",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RotationMatrix<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RotationMatrix<double>::operator*),
           py::arg("v_B"))
      .def("__imul__",
           static_cast<RotationMatrix<double> &(
               RotationMatrix<double>::*)(RotationMatrix<double> const &)>(
               &RotationMatrix<double>::operator*=),
           py::arg("other"));

  py::class_<RotationMatrix<drake::AutoDiffXd>>
      PyRotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "RotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyRotationMatrix_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def(py::init<
               Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<Eigen::Ref<
               ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
               Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("R"))
      .def(py::init<::Eigen::Quaternion<drake::AutoDiffXd, 0> const &>(),
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
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
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
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsOrthonormal(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R,
              double tolerance) { return self.IsValid(R, tolerance); })
      .def("IsValid",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.IsValid(R);
           })
      .def("IsValid",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::IsValid))
      .def("MakeFromOrthonormalColumns",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bx,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &By,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Bz) {
             return self.MakeFromOrthonormalColumns(Bx, By, Bz);
           })
      .def("MakeFromOrthonormalRows",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ax,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Ay,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &Az) {
             return self.MakeFromOrthonormalRows(Ax, Ay, Az);
           })
      .def(
          "MakeXRotation",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<drake::AutoDiffXd const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &theta) {
            return self.MakeXRotation(theta);
          })
      .def(
          "MakeYRotation",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<drake::AutoDiffXd const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &theta) {
            return self.MakeYRotation(theta);
          })
      .def(
          "MakeZRotation",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             Eigen::Ref<drake::AutoDiffXd const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &theta) {
            return self.MakeZRotation(theta);
          })
      .def("ProjectToRotationMatrix",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M,
              Eigen::Ref<drake::AutoDiffXd *, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  quality_factor) {
             return self.ProjectToRotationMatrix(M, quality_factor);
           })
      .def("ToAngleAxis", static_cast<::Eigen::AngleAxis<drake::AutoDiffXd> (
                              RotationMatrix<drake::AutoDiffXd>::*)() const>(
                              &RotationMatrix<drake::AutoDiffXd>::ToAngleAxis))
      .def("ToQuaternion",
           static_cast<::Eigen::Quaternion<drake::AutoDiffXd, 0> (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::ToQuaternion))
      .def(
          "ToQuaternion",
          [](RotationMatrix<drake::AutoDiffXd> &self,
             ::Eigen::Ref<const Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3>,
                          0, Eigen::OuterStride<-1>> const &M) {
            return self.ToQuaternion(M);
          })
      .def("ToQuaternionAsVector4",
           static_cast<::Eigen::Matrix<drake::AutoDiffXd, 4, 1, 0, 4, 1> (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::ToQuaternionAsVector4))
      .def("ToQuaternionAsVector4",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &M) {
             return self.ToQuaternionAsVector4(M);
           })
      .def("col",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3>, 3, 1,
               true> const (RotationMatrix<drake::AutoDiffXd>::*)(int) const>(
               &RotationMatrix<drake::AutoDiffXd>::col),
           py::arg("index"))
      .def_static("get_internal_tolerance_for_orthonormality",
                  static_cast<double (*)()>(
                      &RotationMatrix<drake::AutoDiffXd>::
                          get_internal_tolerance_for_orthonormality))
      .def("inverse", static_cast<RotationMatrix<drake::AutoDiffXd> (
                          RotationMatrix<drake::AutoDiffXd>::*)() const>(
                          &RotationMatrix<drake::AutoDiffXd>::inverse))
      .def(
          "matrix",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &(
              RotationMatrix<drake::AutoDiffXd>::*)() const>(
              &RotationMatrix<drake::AutoDiffXd>::matrix),
          py::return_value_policy::reference_internal)
      .def("row",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3>, 1, 3,
               false> const (RotationMatrix<drake::AutoDiffXd>::*)(int) const>(
               &RotationMatrix<drake::AutoDiffXd>::row),
           py::arg("index"))
      .def("set",
           [](RotationMatrix<drake::AutoDiffXd> &self,
              Eigen::Ref<
                  ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
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
      .def(
          "__mul__",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> (
              RotationMatrix<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RotationMatrix<drake::AutoDiffXd>::operator*),
          py::arg("v_B"))
      .def("__imul__",
           static_cast<RotationMatrix<drake::AutoDiffXd> &(
               RotationMatrix<drake::AutoDiffXd>::
                   *)(RotationMatrix<drake::AutoDiffXd> const &)>(
               &RotationMatrix<drake::AutoDiffXd>::operator*=),
           py::arg("other"));
}
