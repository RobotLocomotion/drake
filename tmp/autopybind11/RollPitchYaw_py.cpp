#include "drake/math/rotation_matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::math;

namespace py = pybind11;
void apb11_pydrake_RollPitchYaw_py_register(py::module &m) {
  py::class_<RollPitchYaw<double>> PyRollPitchYaw_double(m,
                                                         "RollPitchYaw_double");

  PyRollPitchYaw_double
      .def(py::init<RollPitchYaw<double> const &>(), py::arg("arg0"))
      .def(py::init<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &>(),
           py::arg("rpy"))
      .def(py::init<double const &, double const &, double const &>(),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def(py::init<RotationMatrix<double> const &>(), py::arg("R"))
      .def(py::init<::Eigen::Quaternion<double, 0> const &>(),
           py::arg("quaternion"))
      .def("CalcAngularVelocityInChildFromRpyDt",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RollPitchYaw<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RollPitchYaw<double>::CalcAngularVelocityInChildFromRpyDt),
           py::arg("rpyDt"))
      .def("CalcAngularVelocityInParentFromRpyDt",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RollPitchYaw<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RollPitchYaw<double>::CalcAngularVelocityInParentFromRpyDt),
           py::arg("rpyDt"))
      .def("CalcMatrixRelatingRpyDtToAngularVelocityInParent",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const (
               RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<
                   double>::CalcMatrixRelatingRpyDtToAngularVelocityInParent))
      .def("CalcRotationMatrixDt",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> (
               RollPitchYaw<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RollPitchYaw<double>::CalcRotationMatrixDt),
           py::arg("rpyDt"))
      .def("CalcRpyDDtFromAngularAccelInChild",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RollPitchYaw<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &,
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RollPitchYaw<double>::CalcRpyDDtFromAngularAccelInChild),
           py::arg("rpyDt"), py::arg("alpha_AD_D"))
      .def("CalcRpyDDtFromRpyDtAndAngularAccelInParent",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RollPitchYaw<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &,
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RollPitchYaw<
                   double>::CalcRpyDDtFromRpyDtAndAngularAccelInParent),
           py::arg("rpyDt"), py::arg("alpha_AD_A"))
      .def("CalcRpyDtFromAngularVelocityInParent",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> (
               RollPitchYaw<double>::*)(
               ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &) const>(
               &RollPitchYaw<double>::CalcRpyDtFromAngularVelocityInParent),
           py::arg("w_AD_A"))
      .def_static("DoesCosPitchAngleViolateGimbalLockTolerance",
                  static_cast<::drake::scalar_predicate<double>::type (*)(
                      double const &)>(
                      &RollPitchYaw<
                          double>::DoesCosPitchAngleViolateGimbalLockTolerance),
                  py::arg("cos_pitch_angle"))
      .def("DoesPitchAngleViolateGimbalLockTolerance",
           static_cast<::drake::scalar_predicate<double>::type (
               RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<double>::DoesPitchAngleViolateGimbalLockTolerance))
      .def_static("GimbalLockPitchAngleTolerance",
                  static_cast<double (*)()>(
                      &RollPitchYaw<double>::GimbalLockPitchAngleTolerance))
      .def("IsNearlyEqualTo",
           static_cast<::drake::scalar_predicate<double>::type (
               RollPitchYaw<double>::*)(RollPitchYaw<double> const &, double)
                           const>(&RollPitchYaw<double>::IsNearlyEqualTo),
           py::arg("other"), py::arg("tolerance"))
      .def("IsNearlySameOrientation",
           static_cast<::drake::scalar_predicate<double>::type (
               RollPitchYaw<double>::*)(RollPitchYaw<double> const &, double)
                           const>(
               &RollPitchYaw<double>::IsNearlySameOrientation),
           py::arg("other"), py::arg("tolerance"))
      .def("IsRollPitchYawInCanonicalRange",
           static_cast<::drake::scalar_predicate<double>::type (
               RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<double>::IsRollPitchYawInCanonicalRange))
      .def_static("IsValid",
                  static_cast<::drake::scalar_predicate<double>::type (*)(
                      ::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &)>(
                      &RollPitchYaw<double>::IsValid),
                  py::arg("rpy"))
      .def("SetFromQuaternion",
           static_cast<void (RollPitchYaw<double>::*)(
               ::Eigen::Quaternion<double, 0> const &)>(
               &RollPitchYaw<double>::SetFromQuaternion),
           py::arg("quaternion"))
      .def("SetFromRotationMatrix",
           static_cast<void (RollPitchYaw<double>::*)(
               RotationMatrix<double> const &)>(
               &RollPitchYaw<double>::SetFromRotationMatrix),
           py::arg("R"))
      .def("ToMatrix3ViaRotationMatrix",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> (
               RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<double>::ToMatrix3ViaRotationMatrix))
      .def("ToQuaternion", static_cast<::Eigen::Quaternion<double, 0> (
                               RollPitchYaw<double>::*)() const>(
                               &RollPitchYaw<double>::ToQuaternion))
      .def(
          "ToRotationMatrix",
          static_cast<RotationMatrix<double> (RollPitchYaw<double>::*)() const>(
              &RollPitchYaw<double>::ToRotationMatrix))
      .def("pitch_angle",
           static_cast<double const &(RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<double>::pitch_angle))
      .def("roll_angle",
           static_cast<double const &(RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<double>::roll_angle))
      .def("set",
           static_cast<RollPitchYaw<double> &(
               RollPitchYaw<double>::*)(::Eigen::Matrix<double, 3, 1, 0, 3,
                                                        1> const &)>(
               &RollPitchYaw<double>::set),
           py::arg("rpy"))
      .def("set",
           static_cast<RollPitchYaw<double> &(
               RollPitchYaw<double>::*)(double const &, double const &,
                                        double const &)>(
               &RollPitchYaw<double>::set),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def("set_pitch_angle",
           static_cast<void (RollPitchYaw<double>::*)(double const &)>(
               &RollPitchYaw<double>::set_pitch_angle),
           py::arg("p"))
      .def("set_roll_angle",
           static_cast<void (RollPitchYaw<double>::*)(double const &)>(
               &RollPitchYaw<double>::set_roll_angle),
           py::arg("r"))
      .def("set_yaw_angle",
           static_cast<void (RollPitchYaw<double>::*)(double const &)>(
               &RollPitchYaw<double>::set_yaw_angle),
           py::arg("y"))
      .def("vector",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &(
               RollPitchYaw<double>::*)() const>(&RollPitchYaw<double>::vector))
      .def("yaw_angle",
           static_cast<double const &(RollPitchYaw<double>::*)() const>(
               &RollPitchYaw<double>::yaw_angle))

      ;

  py::class_<RollPitchYaw<drake::AutoDiffXd>>
      PyRollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "RollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyRollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def(py::init<RollPitchYaw<drake::AutoDiffXd> const &>(), py::arg("arg0"))
      .def(
          py::init<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &>(),
          py::arg("rpy"))
      .def(py::init<drake::AutoDiffXd const &, drake::AutoDiffXd const &,
                    drake::AutoDiffXd const &>(),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def(py::init<RotationMatrix<drake::AutoDiffXd> const &>(), py::arg("R"))
      .def(py::init<::Eigen::Quaternion<drake::AutoDiffXd, 0> const &>(),
           py::arg("quaternion"))
      .def(
          "CalcAngularVelocityInChildFromRpyDt",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> (
              RollPitchYaw<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RollPitchYaw<
                  drake::AutoDiffXd>::CalcAngularVelocityInChildFromRpyDt),
          py::arg("rpyDt"))
      .def(
          "CalcAngularVelocityInParentFromRpyDt",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> (
              RollPitchYaw<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RollPitchYaw<
                  drake::AutoDiffXd>::CalcAngularVelocityInParentFromRpyDt),
          py::arg("rpyDt"))
      .def("CalcMatrixRelatingRpyDtToAngularVelocityInParent",
           static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> const (
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::
                   CalcMatrixRelatingRpyDtToAngularVelocityInParent))
      .def(
          "CalcRotationMatrixDt",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> (
              RollPitchYaw<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RollPitchYaw<drake::AutoDiffXd>::CalcRotationMatrixDt),
          py::arg("rpyDt"))
      .def(
          "CalcRpyDDtFromAngularAccelInChild",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> (
              RollPitchYaw<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &,
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RollPitchYaw<
                  drake::AutoDiffXd>::CalcRpyDDtFromAngularAccelInChild),
          py::arg("rpyDt"), py::arg("alpha_AD_D"))
      .def(
          "CalcRpyDDtFromRpyDtAndAngularAccelInParent",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> (
              RollPitchYaw<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &,
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RollPitchYaw<drake::AutoDiffXd>::
                  CalcRpyDDtFromRpyDtAndAngularAccelInParent),
          py::arg("rpyDt"), py::arg("alpha_AD_A"))
      .def(
          "CalcRpyDtFromAngularVelocityInParent",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> (
              RollPitchYaw<drake::AutoDiffXd>::*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &) const>(
              &RollPitchYaw<
                  drake::AutoDiffXd>::CalcRpyDtFromAngularVelocityInParent),
          py::arg("w_AD_A"))
      .def_static(
          "DoesCosPitchAngleViolateGimbalLockTolerance",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (*)(
              drake::AutoDiffXd const &)>(
              &RollPitchYaw<drake::AutoDiffXd>::
                  DoesCosPitchAngleViolateGimbalLockTolerance),
          py::arg("cos_pitch_angle"))
      .def(
          "DoesPitchAngleViolateGimbalLockTolerance",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
              RollPitchYaw<drake::AutoDiffXd>::*)() const>(
              &RollPitchYaw<
                  drake::AutoDiffXd>::DoesPitchAngleViolateGimbalLockTolerance))
      .def_static(
          "GimbalLockPitchAngleTolerance",
          static_cast<double (*)()>(
              &RollPitchYaw<drake::AutoDiffXd>::GimbalLockPitchAngleTolerance))
      .def("IsNearlyEqualTo",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RollPitchYaw<drake::AutoDiffXd>::*)(
               RollPitchYaw<drake::AutoDiffXd> const &, double) const>(
               &RollPitchYaw<drake::AutoDiffXd>::IsNearlyEqualTo),
           py::arg("other"), py::arg("tolerance"))
      .def("IsNearlySameOrientation",
           static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
               RollPitchYaw<drake::AutoDiffXd>::*)(
               RollPitchYaw<drake::AutoDiffXd> const &, double) const>(
               &RollPitchYaw<drake::AutoDiffXd>::IsNearlySameOrientation),
           py::arg("other"), py::arg("tolerance"))
      .def(
          "IsRollPitchYawInCanonicalRange",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
              RollPitchYaw<drake::AutoDiffXd>::*)() const>(
              &RollPitchYaw<drake::AutoDiffXd>::IsRollPitchYawInCanonicalRange))
      .def_static(
          "IsValid",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (*)(
              ::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &)>(
              &RollPitchYaw<drake::AutoDiffXd>::IsValid),
          py::arg("rpy"))
      .def("SetFromQuaternion",
           static_cast<void (RollPitchYaw<drake::AutoDiffXd>::*)(
               ::Eigen::Quaternion<drake::AutoDiffXd, 0> const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::SetFromQuaternion),
           py::arg("quaternion"))
      .def("SetFromRotationMatrix",
           static_cast<void (RollPitchYaw<drake::AutoDiffXd>::*)(
               RotationMatrix<drake::AutoDiffXd> const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::SetFromRotationMatrix),
           py::arg("R"))
      .def("ToMatrix3ViaRotationMatrix",
           static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 3, 0, 3, 3> (
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::ToMatrix3ViaRotationMatrix))
      .def("ToQuaternion",
           static_cast<::Eigen::Quaternion<drake::AutoDiffXd, 0> (
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::ToQuaternion))
      .def("ToRotationMatrix",
           static_cast<RotationMatrix<drake::AutoDiffXd> (
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::ToRotationMatrix))
      .def("pitch_angle", static_cast<drake::AutoDiffXd const &(
                              RollPitchYaw<drake::AutoDiffXd>::*)() const>(
                              &RollPitchYaw<drake::AutoDiffXd>::pitch_angle))
      .def("roll_angle", static_cast<drake::AutoDiffXd const &(
                             RollPitchYaw<drake::AutoDiffXd>::*)() const>(
                             &RollPitchYaw<drake::AutoDiffXd>::roll_angle))
      .def("set",
           static_cast<RollPitchYaw<drake::AutoDiffXd> &(
               RollPitchYaw<drake::AutoDiffXd>::*)(::Eigen::Matrix<
                                                   drake::AutoDiffXd, 3, 1, 0,
                                                   3, 1> const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::set),
           py::arg("rpy"))
      .def("set",
           static_cast<RollPitchYaw<drake::AutoDiffXd> &(
               RollPitchYaw<drake::AutoDiffXd>::*)(drake::AutoDiffXd const &,
                                                   drake::AutoDiffXd const &,
                                                   drake::AutoDiffXd const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::set),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def("set_pitch_angle",
           static_cast<void (RollPitchYaw<drake::AutoDiffXd>::*)(
               drake::AutoDiffXd const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::set_pitch_angle),
           py::arg("p"))
      .def("set_roll_angle",
           static_cast<void (RollPitchYaw<drake::AutoDiffXd>::*)(
               drake::AutoDiffXd const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::set_roll_angle),
           py::arg("r"))
      .def("set_yaw_angle",
           static_cast<void (RollPitchYaw<drake::AutoDiffXd>::*)(
               drake::AutoDiffXd const &)>(
               &RollPitchYaw<drake::AutoDiffXd>::set_yaw_angle),
           py::arg("y"))
      .def(
          "vector",
          static_cast<::Eigen::Matrix<drake::AutoDiffXd, 3, 1, 0, 3, 1> const &(
              RollPitchYaw<drake::AutoDiffXd>::*)() const>(
              &RollPitchYaw<drake::AutoDiffXd>::vector))
      .def("yaw_angle", static_cast<drake::AutoDiffXd const &(
                            RollPitchYaw<drake::AutoDiffXd>::*)() const>(
                            &RollPitchYaw<drake::AutoDiffXd>::yaw_angle))

      ;
}
