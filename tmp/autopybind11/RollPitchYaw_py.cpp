#include "drake/math/rotation_matrix.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::math;

namespace py = pybind11;
void apb11_pydrake_RollPitchYaw_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using PyRollPitchYaw_double_0 = double;

  py::class_<RollPitchYaw<PyRollPitchYaw_double_0>> PyRollPitchYaw_double(
      m, "RollPitchYaw_double");

  PyRollPitchYaw_double
      .def(py::init<RollPitchYaw<PyRollPitchYaw_double_0> const &>(),
           py::arg("arg0"))
      .def(py::init<
               Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("rpy"))
      .def(py::init<double const &, double const &, double const &>(),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def(py::init<RotationMatrix<PyRollPitchYaw_double_0> const &>(),
           py::arg("R"))
      .def(py::init<::Eigen::Quaternion<double, 0> const &>(),
           py::arg("quaternion"))
      .def(
          "CalcAngularVelocityInChildFromRpyDt",
          [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
             Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpyDt) {
            return self.CalcAngularVelocityInChildFromRpyDt(rpyDt);
          })
      .def(
          "CalcAngularVelocityInParentFromRpyDt",
          [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
             Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpyDt) {
            return self.CalcAngularVelocityInParentFromRpyDt(rpyDt);
          })
      .def("CalcMatrixRelatingRpyDtToAngularVelocityInParent",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> const (
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::
                   CalcMatrixRelatingRpyDtToAngularVelocityInParent))
      .def(
          "CalcRotationMatrixDt",
          [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
             Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpyDt) {
            return self.CalcRotationMatrixDt(rpyDt);
          })
      .def("CalcRpyDDtFromAngularAccelInChild",
           [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpyDt,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &alpha_AD_D) {
             return self.CalcRpyDDtFromAngularAccelInChild(rpyDt, alpha_AD_D);
           })
      .def("CalcRpyDDtFromRpyDtAndAngularAccelInParent",
           [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpyDt,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &alpha_AD_A) {
             return self.CalcRpyDDtFromRpyDtAndAngularAccelInParent(rpyDt,
                                                                    alpha_AD_A);
           })
      .def("CalcRpyDtFromAngularVelocityInParent",
           [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &w_AD_A) {
             return self.CalcRpyDtFromAngularVelocityInParent(w_AD_A);
           })
      .def_static(
          "DoesCosPitchAngleViolateGimbalLockTolerance",
          static_cast<::drake::scalar_predicate<PyRollPitchYaw_double_0>::type (
                  *)(double const &)>(
              &RollPitchYaw<PyRollPitchYaw_double_0>::
                  DoesCosPitchAngleViolateGimbalLockTolerance),
          py::arg("cos_pitch_angle"))
      .def(
          "DoesPitchAngleViolateGimbalLockTolerance",
          static_cast<::drake::scalar_predicate<PyRollPitchYaw_double_0>::type (
              RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
              &RollPitchYaw<PyRollPitchYaw_double_0>::
                  DoesPitchAngleViolateGimbalLockTolerance))
      .def_static(
          "GimbalLockPitchAngleTolerance",
          static_cast<double (*)()>(
              &RollPitchYaw<
                  PyRollPitchYaw_double_0>::GimbalLockPitchAngleTolerance))
      .def(
          "IsNearlyEqualTo",
          static_cast<::drake::scalar_predicate<PyRollPitchYaw_double_0>::type (
              RollPitchYaw<PyRollPitchYaw_double_0>::*)(
              RollPitchYaw<PyRollPitchYaw_double_0> const &, double) const>(
              &RollPitchYaw<PyRollPitchYaw_double_0>::IsNearlyEqualTo),
          py::arg("other"), py::arg("tolerance"))
      .def(
          "IsNearlySameOrientation",
          static_cast<::drake::scalar_predicate<PyRollPitchYaw_double_0>::type (
              RollPitchYaw<PyRollPitchYaw_double_0>::*)(
              RollPitchYaw<PyRollPitchYaw_double_0> const &, double) const>(
              &RollPitchYaw<PyRollPitchYaw_double_0>::IsNearlySameOrientation),
          py::arg("other"), py::arg("tolerance"))
      .def(
          "IsRollPitchYawInCanonicalRange",
          static_cast<::drake::scalar_predicate<PyRollPitchYaw_double_0>::type (
              RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
              &RollPitchYaw<
                  PyRollPitchYaw_double_0>::IsRollPitchYawInCanonicalRange))
      .def("IsValid",
           [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpy) {
             return self.IsValid(rpy);
           })
      .def("SetFromQuaternion",
           [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
              ::Eigen::Quaternion<double, 0> const &quaternion) {
             return self.SetFromQuaternion(quaternion);
           })
      .def("SetFromRotationMatrix",
           static_cast<void (RollPitchYaw<PyRollPitchYaw_double_0>::*)(
               RotationMatrix<PyRollPitchYaw_double_0> const &)>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::SetFromRotationMatrix),
           py::arg("R"))
      .def("ToMatrix3ViaRotationMatrix",
           static_cast<::Eigen::Matrix<double, 3, 3, 0, 3, 3> (
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<
                   PyRollPitchYaw_double_0>::ToMatrix3ViaRotationMatrix))
      .def("ToQuaternion",
           static_cast<::Eigen::Quaternion<double, 0> (
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::ToQuaternion))
      .def("ToRotationMatrix",
           static_cast<RotationMatrix<PyRollPitchYaw_double_0> (
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::ToRotationMatrix))
      .def("pitch_angle",
           static_cast<double const &(
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::pitch_angle))
      .def("roll_angle",
           static_cast<double const &(
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::roll_angle))
      .def("set",
           [](RollPitchYaw<PyRollPitchYaw_double_0> &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &rpy) {
             return self.set(rpy);
           })
      .def("set",
           static_cast<RollPitchYaw<PyRollPitchYaw_double_0> &(
               RollPitchYaw<PyRollPitchYaw_double_0>::*)(double const &,
                                                         double const &,
                                                         double const &)>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::set),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def("set_pitch_angle",
           static_cast<void (RollPitchYaw<PyRollPitchYaw_double_0>::*)(
               double const &)>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::set_pitch_angle),
           py::arg("p"))
      .def("set_roll_angle",
           static_cast<void (RollPitchYaw<PyRollPitchYaw_double_0>::*)(
               double const &)>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::set_roll_angle),
           py::arg("r"))
      .def("set_yaw_angle",
           static_cast<void (RollPitchYaw<PyRollPitchYaw_double_0>::*)(
               double const &)>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::set_yaw_angle),
           py::arg("y"))
      .def("vector",
           static_cast<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &(
               RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
               &RollPitchYaw<PyRollPitchYaw_double_0>::vector),
           py::return_value_policy::reference_internal)
      .def("yaw_angle", static_cast<double const &(
                            RollPitchYaw<PyRollPitchYaw_double_0>::*)() const>(
                            &RollPitchYaw<PyRollPitchYaw_double_0>::yaw_angle))

      ;

  using PyRollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd_0 =
      drake::AutoDiffXd;

  py::class_<RollPitchYaw<drake::AutoDiffXd>>
      PyRollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd(
          m, "RollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd");

  PyRollPitchYaw_Eigen_AutoDiffScalar_Eigen_VectorXd
      .def(py::init<RollPitchYaw<drake::AutoDiffXd> const &>(), py::arg("arg0"))
      .def(
          py::init<
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
          py::arg("rpy"))
      .def(py::init<
               Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,
               Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &,
               Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
      .def(py::init<
               Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                          Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &>(),
           py::arg("R"))
      .def(py::init<::Eigen::Quaternion<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        0> const &>(),
           py::arg("quaternion"))
      .def("CalcAngularVelocityInChildFromRpyDt",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &rpyDt) {
             return self.CalcAngularVelocityInChildFromRpyDt(rpyDt);
           })
      .def("CalcAngularVelocityInParentFromRpyDt",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &rpyDt) {
             return self.CalcAngularVelocityInParentFromRpyDt(rpyDt);
           })
      .def("CalcMatrixRelatingRpyDtToAngularVelocityInParent",
           static_cast<::Eigen::Matrix<
               Eigen::AutoDiffScalar<Eigen::VectorXd>, 3, 3, 0, 3,
               3> const (RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::
                   CalcMatrixRelatingRpyDtToAngularVelocityInParent))
      .def("CalcRotationMatrixDt",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &rpyDt) { return self.CalcRotationMatrixDt(rpyDt); })
      .def("CalcRpyDDtFromAngularAccelInChild",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &rpyDt,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &alpha_AD_D) {
             return self.CalcRpyDDtFromAngularAccelInChild(rpyDt, alpha_AD_D);
           })
      .def("CalcRpyDDtFromRpyDtAndAngularAccelInParent",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &rpyDt,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &alpha_AD_A) {
             return self.CalcRpyDDtFromRpyDtAndAngularAccelInParent(rpyDt,
                                                                    alpha_AD_A);
           })
      .def("CalcRpyDtFromAngularVelocityInParent",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &w_AD_A) {
             return self.CalcRpyDtFromAngularVelocityInParent(w_AD_A);
           })
      .def("DoesCosPitchAngleViolateGimbalLockTolerance",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &cos_pitch_angle) {
             return self.DoesCosPitchAngleViolateGimbalLockTolerance(
                 cos_pitch_angle);
           })
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
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              RollPitchYaw<drake::AutoDiffXd> const &other, double tolerance) {
             return self.IsNearlyEqualTo(other, tolerance);
           })
      .def("IsNearlySameOrientation",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              RollPitchYaw<drake::AutoDiffXd> const &other, double tolerance) {
             return self.IsNearlySameOrientation(other, tolerance);
           })
      .def(
          "IsRollPitchYawInCanonicalRange",
          static_cast<::drake::scalar_predicate<drake::AutoDiffXd>::type (
              RollPitchYaw<drake::AutoDiffXd>::*)() const>(
              &RollPitchYaw<drake::AutoDiffXd>::IsRollPitchYawInCanonicalRange))
      .def("IsValid",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                         3, 1, 0, 3, 1> const &,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  &rpy) { return self.IsValid(rpy); })
      .def("SetFromQuaternion",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              ::Eigen::Quaternion<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                  0> const &quaternion) {
             return self.SetFromQuaternion(quaternion);
           })
      .def("SetFromRotationMatrix",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<RotationMatrix<drake::AutoDiffXd> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &R) {
             return self.SetFromRotationMatrix(R);
           })
      .def("ToMatrix3ViaRotationMatrix",
           static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                       3, 3, 0, 3, 3> (
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::ToMatrix3ViaRotationMatrix))
      .def("ToQuaternion",
           static_cast<
               ::Eigen::Quaternion<Eigen::AutoDiffScalar<Eigen::VectorXd>, 0> (
                   RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::ToQuaternion))
      .def("ToRotationMatrix",
           static_cast<RotationMatrix<drake::AutoDiffXd> (
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::ToRotationMatrix))
      .def("pitch_angle",
           static_cast<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &(
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::pitch_angle),
           py::return_value_policy::reference_internal)
      .def("roll_angle",
           static_cast<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &(
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::roll_angle),
           py::return_value_policy::reference_internal)
      .def(
          "set",
          [](RollPitchYaw<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                        3, 1, 0, 3, 1> const &,
                        0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                 &rpy) { return self.set(rpy); },
          py::return_value_policy::reference_internal)
      .def(
          "set",
          [](RollPitchYaw<drake::AutoDiffXd> &self,
             Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &roll,
             Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &pitch,
             Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                        Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &yaw) {
            return self.set(roll, pitch, yaw);
          },
          py::return_value_policy::reference_internal)
      .def("set_pitch_angle",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &p) {
             return self.set_pitch_angle(p);
           })
      .def("set_roll_angle",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &r) {
             return self.set_roll_angle(r);
           })
      .def("set_yaw_angle",
           [](RollPitchYaw<drake::AutoDiffXd> &self,
              Eigen::Ref<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &y) {
             return self.set_yaw_angle(y);
           })
      .def("vector",
           static_cast<::Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>,
                                       3, 1, 0, 3, 1> const
                           &(RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::vector),
           py::return_value_policy::reference_internal)
      .def("yaw_angle",
           static_cast<::Eigen::AutoDiffScalar<Eigen::VectorXd> const &(
               RollPitchYaw<drake::AutoDiffXd>::*)() const>(
               &RollPitchYaw<drake::AutoDiffXd>::yaw_angle),
           py::return_value_policy::reference_internal)

      ;
}
