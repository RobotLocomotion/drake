#include "drake/math/rotation_matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::math;

namespace py = pybind11;
void apb11_pydrake_RotationMatrix_py_register(py::module &m) {
  py::class_<RotationMatrix<double>> PyRotationMatrix_double(
      m, "RotationMatrix_double");

  PyRotationMatrix_double
      .def(py::init<RotationMatrix<double> const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &>(),
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
      .def_static("GetMeasureOfOrthonormality",
                  static_cast<double (*)(
                      ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &)>(
                      &RotationMatrix<double>::GetMeasureOfOrthonormality),
                  py::arg("R"))
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
      .def_static("IsOrthonormal",
                  static_cast<::drake::scalar_predicate<double>::type (*)(
                      ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, double)>(
                      &RotationMatrix<double>::IsOrthonormal),
                  py::arg("R"), py::arg("tolerance"))
      .def_static("IsValid",
                  static_cast<::drake::scalar_predicate<double>::type (*)(
                      ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, double)>(
                      &RotationMatrix<double>::IsValid),
                  py::arg("R"), py::arg("tolerance"))
      .def_static("IsValid",
                  static_cast<::drake::scalar_predicate<double>::type (*)(
                      ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &)>(
                      &RotationMatrix<double>::IsValid),
                  py::arg("R"))
      .def("IsValid", static_cast<::drake::scalar_predicate<double>::type (
                          RotationMatrix<double>::*)() const>(
                          &RotationMatrix<double>::IsValid))
      .def_static("MakeFromOrthonormalColumns",
                  static_cast<RotationMatrix<double> (*)(
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &,
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &,
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &)>(
                      &RotationMatrix<double>::MakeFromOrthonormalColumns),
                  py::arg("Bx"), py::arg("By"), py::arg("Bz"))
      .def_static("MakeFromOrthonormalRows",
                  static_cast<RotationMatrix<double> (*)(
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &,
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &,
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &)>(
                      &RotationMatrix<double>::MakeFromOrthonormalRows),
                  py::arg("Ax"), py::arg("Ay"), py::arg("Az"))
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
      .def_static(
          "ProjectToRotationMatrix",
          static_cast<RotationMatrix<double> (*)(
              ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &, double *)>(
              &RotationMatrix<double>::ProjectToRotationMatrix),
          py::arg("M"), py::arg("quality_factor") = (double *)nullptr)
      .def("ToAngleAxis",
           static_cast<::Eigen::AngleAxis<double> (RotationMatrix<double>::*)()
                           const>(&RotationMatrix<double>::ToAngleAxis))
      .def("ToQuaternion", static_cast<::Eigen::Quaternion<double, 0> (
                               RotationMatrix<double>::*)() const>(
                               &RotationMatrix<double>::ToQuaternion))
      .def_static("ToQuaternion",
                  static_cast<::Eigen::Quaternion<double, 0> (*)(
                      ::Eigen::Ref<const Eigen::Matrix<double, 3, 3, 0, 3, 3>,
                                   0, Eigen::OuterStride<-1>> const &)>(
                      &RotationMatrix<double>::ToQuaternion),
                  py::arg("M"))
      .def("ToQuaternionAsVector4",
           static_cast<::Eigen::Matrix<double, 4, 1, 0, 4, 1> (
               RotationMatrix<double>::*)() const>(
               &RotationMatrix<double>::ToQuaternionAsVector4))
      .def_static("ToQuaternionAsVector4",
                  static_cast<::Eigen::Matrix<double, 4, 1, 0, 4, 1> (*)(
                      ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &)>(
                      &RotationMatrix<double>::ToQuaternionAsVector4),
                  py::arg("M"))
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
      .def("matrix", static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &(
                         RotationMatrix<double>::*)() const>(
                         &RotationMatrix<double>::matrix))
      .def("row",
           static_cast<
               ::Eigen::Block<const Eigen::Matrix<double, 3, 3, 0, 3, 3>, 1, 3,
                              false> const (RotationMatrix<double>::*)(int)
                   const>(&RotationMatrix<double>::row),
           py::arg("index"))
      .def("set",
           static_cast<void (RotationMatrix<double>::*)(
               ::Eigen::Matrix<double, 3, 3, 0, 3, 3> const &)>(
               &RotationMatrix<double>::set),
           py::arg("R"))
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
      .def(py::init<RotationMatrix<drake::AutoDiffXd> const &>(),
           py::arg("arg0"))
      .def(py::init<>())
      .def(
          py::init<::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &>(),
          py::arg("R"))
      .def(py::init<::Eigen::Quaternion<drake::AutoDiffXd, 0> const &>(),
           py::arg("quaternion"))
      .def(py::init<::Eigen::AngleAxis<drake::AutoDiffXd> const &>(),
           py::arg("theta_lambda"))
      .def(py::init<RollPitchYaw<drake::AutoDiffXd> const &>(), py::arg("rpy"))
      .def(
          "GetMaximumAbsoluteDifference",
          static_cast<drake::AutoDiffXd (RotationMatrix<drake::AutoDiffXd>::*)(
              RotationMatrix<drake::AutoDiffXd> const &) const>(
              &RotationMatrix<drake::AutoDiffXd>::GetMaximumAbsoluteDifference),
          py::arg("other"))
      .def_static(
          "GetMeasureOfOrthonormality",
          static_cast<drake::AutoDiffXd (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &)>(
              &RotationMatrix<drake::AutoDiffXd>::GetMeasureOfOrthonormality),
          py::arg("R"))
      .def_static("Identity",
                  static_cast<RotationMatrix<drake::AutoDiffXd> const &(*)()>(
                      &RotationMatrix<drake::AutoDiffXd>::Identity))
      .def("IsExactlyEqualTo",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)(
               RotationMatrix<drake::AutoDiffXd> const &) const>(
               &RotationMatrix<drake::AutoDiffXd>::IsExactlyEqualTo),
           py::arg("other"))
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
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)(
               RotationMatrix<drake::AutoDiffXd> const &, double) const>(
               &RotationMatrix<drake::AutoDiffXd>::IsNearlyEqualTo),
           py::arg("other"), py::arg("tolerance"))
      .def_static(
          "IsOrthonormal",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &,
              double)>(&RotationMatrix<drake::AutoDiffXd>::IsOrthonormal),
          py::arg("R"), py::arg("tolerance"))
      .def_static(
          "IsValid",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &,
              double)>(&RotationMatrix<drake::AutoDiffXd>::IsValid),
          py::arg("R"), py::arg("tolerance"))
      .def_static(
          "IsValid",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &)>(
              &RotationMatrix<drake::AutoDiffXd>::IsValid),
          py::arg("R"))
      .def("IsValid",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::IsValid))
      .def_static(
          "MakeFromOrthonormalColumns",
          static_cast<RotationMatrix<drake::AutoDiffXd> (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &,
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &,
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &)>(
              &RotationMatrix<drake::AutoDiffXd>::MakeFromOrthonormalColumns),
          py::arg("Bx"), py::arg("By"), py::arg("Bz"))
      .def_static(
          "MakeFromOrthonormalRows",
          static_cast<RotationMatrix<drake::AutoDiffXd> (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &,
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &,
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &)>(
              &RotationMatrix<drake::AutoDiffXd>::MakeFromOrthonormalRows),
          py::arg("Ax"), py::arg("Ay"), py::arg("Az"))
      .def_static("MakeXRotation",
                  static_cast<RotationMatrix<drake::AutoDiffXd> (*)(
                      drake::AutoDiffXd const &)>(
                      &RotationMatrix<drake::AutoDiffXd>::MakeXRotation),
                  py::arg("theta"))
      .def_static("MakeYRotation",
                  static_cast<RotationMatrix<drake::AutoDiffXd> (*)(
                      drake::AutoDiffXd const &)>(
                      &RotationMatrix<drake::AutoDiffXd>::MakeYRotation),
                  py::arg("theta"))
      .def_static("MakeZRotation",
                  static_cast<RotationMatrix<drake::AutoDiffXd> (*)(
                      drake::AutoDiffXd const &)>(
                      &RotationMatrix<drake::AutoDiffXd>::MakeZRotation),
                  py::arg("theta"))
      .def_static(
          "ProjectToRotationMatrix",
          static_cast<RotationMatrix<drake::AutoDiffXd> (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &,
              drake::AutoDiffXd *)>(
              &RotationMatrix<drake::AutoDiffXd>::ProjectToRotationMatrix),
          py::arg("M"),
          py::arg("quality_factor") = (drake::AutoDiffXd *)nullptr)
      .def("ToAngleAxis", static_cast<::Eigen::AngleAxis<drake::AutoDiffXd> (
                              RotationMatrix<drake::AutoDiffXd>::*)() const>(
                              &RotationMatrix<drake::AutoDiffXd>::ToAngleAxis))
      .def("ToQuaternion",
           static_cast<::Eigen::Quaternion<drake::AutoDiffXd, 0> (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::ToQuaternion))
      .def_static("ToQuaternion",
                  static_cast<::Eigen::Quaternion<drake::AutoDiffXd, 0> (*)(
                      ::Eigen::Ref<
                          const Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3>,
                          0, Eigen::OuterStride<-1>> const &)>(
                      &RotationMatrix<drake::AutoDiffXd>::ToQuaternion),
                  py::arg("M"))
      .def("ToQuaternionAsVector4",
           static_cast<::Eigen::Matrix<drake::AutoDiffXd, 4, 1, 0, 4, 1> (
               RotationMatrix<drake::AutoDiffXd>::*)() const>(
               &RotationMatrix<drake::AutoDiffXd>::ToQuaternionAsVector4))
      .def_static(
          "ToQuaternionAsVector4",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 4, 1, 0, 4, 1> (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &)>(
              &RotationMatrix<drake::AutoDiffXd>::ToQuaternionAsVector4),
          py::arg("M"))
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
              &RotationMatrix<drake::AutoDiffXd>::matrix))
      .def("row",
           static_cast<::Eigen::Block<
               const Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3>, 1, 3,
               false> const (RotationMatrix<drake::AutoDiffXd>::*)(int) const>(
               &RotationMatrix<drake::AutoDiffXd>::row),
           py::arg("index"))
      .def("set",
           static_cast<void (RotationMatrix<drake::AutoDiffXd>::*)(
               ::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const &)>(
               &RotationMatrix<drake::AutoDiffXd>::set),
           py::arg("R"))
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
