#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/tree/quaternion_rate.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(cc, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  py::module::import("pydrake.common.eigen_geometry");

  {
    using Class = QuaternionRate<double>;
    py::class_<Class>(m, "QuaternionRate")
        .def_static(
            "AngularVelocityToQuaternionRateMatrix",
            &Class::AngularVelocityToQuaternionRateMatrix)
        .def_static(
            "QuaternionRateToAngularVelocityMatrix",
            &Class::QuaternionRateToAngularVelocityMatrix);
  }

  // Hack around normalization check for bindings.
  m.def(
      "hack_quaternion",
      [](const Eigen::Vector4d& wxyz) {
        return Eigen::Quaternion<double>(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
      }, py::arg("wxyz"));
}

}  // namespace pydrake
}  // namespace drake
