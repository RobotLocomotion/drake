#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

using Eigen::VectorXd;

using Eigen::VectorXd;

namespace drake {
namespace pydrake {

class AutoDiffContainer {
 public:
  AutoDiffContainer() {
    value_.resize(1, 2);
    value_ << AutoDiffXd(10, (VectorXd(2) << 1, 0).finished()),
        AutoDiffXd(100, (VectorXd(2) << 0, 1).finished());
  }
  MatrixX<AutoDiffXd>& value() { return value_; }

 private:
  MatrixX<AutoDiffXd> value_;
};

PYBIND11_MODULE(autodiffutils_test_util, m) {
  m.doc() = "Utilities for testing Eigen AutoDiff Scalars";

  py::module::import("pydrake.autodiffutils");

  // Implicit argument conversion.
  m.def("autodiff_scalar_pass_through",
        [](const AutoDiffXd& value) { return value; });
  m.def("autodiff_vector_pass_through",
        [](const VectorX<AutoDiffXd>& value) { return value; });
  m.def("autodiff_vector3_pass_through",
      [](const Vector3<AutoDiffXd>& value) { return value; });

  // Reference semantics for AutoDiff.
  py::class_<AutoDiffContainer>(m, "AutoDiffContainer")
      .def(py::init())
      .def("value", &AutoDiffContainer::value, py_reference_internal);

  m.def("autodiff_increment",
        [](Eigen::Ref<MatrixX<AutoDiffXd>> value) { value.array() += 1; });
}

}  // namespace pydrake
}  // namespace drake
