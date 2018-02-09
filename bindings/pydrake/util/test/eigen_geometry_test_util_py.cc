#include <Eigen/Dense>
#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Translation3d;

namespace drake {
namespace pydrake {
namespace {

const double kTolerance = 1e-8;

}  // namespace

PYBIND11_MODULE(eigen_geometry_test_util, m) {
  m.doc() = "Bindings for Eigen geometric types.";

  using T = double;

  m.def("create_isometry", []() {
    return Isometry3<T>::Identity();
  });
  m.def("check_isometry", [](const Isometry3d& X) {
    const T error =
        (X.matrix() - Isometry3<T>::Identity().matrix())
        .array().abs().maxCoeff();
    DRAKE_THROW_UNLESS(error < kTolerance);
  });

  m.def("create_translation", []() {
    return Translation3<T>(Vector3<T>::Zero());
  });
  m.def("check_translation", [](const Translation3<T>& p) {
    const T error = p.vector().array().abs().maxCoeff();
    DRAKE_THROW_UNLESS(error < kTolerance);
  });

  m.def("create_quaternion", []() {
    return Quaternion<T>::Identity();
  });
  m.def("check_quaternion", [](const Quaternion<T>& q) {
    const T error =
        (q.coeffs() - Quaternion<T>::Identity().coeffs())
        .array().abs().maxCoeff();
    DRAKE_THROW_UNLESS(error < kTolerance);
  });
}

}  // namespace pydrake
}  // namespace drake
