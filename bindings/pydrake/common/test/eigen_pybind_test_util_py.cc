#include "drake/bindings/pydrake/common/eigen_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(eigen_pybind_test_util, m) {
  m.doc() = "Bindings for Eigen types.";

  using T = double;

  m.def("takes_returns_matrix_pointer",
      [](drake::EigenPtr<MatrixX<T>> mat) { return mat; });

  m.def("scale_matrix_ptr", [](drake::EigenPtr<MatrixX<T>> mat, T factor) {
    if (mat != nullptr) {
      for (auto it = mat->data(); it != mat->data() + mat->size(); ++it) {
        *it *= factor;
      }
    }

    return mat;
  });

  m.def(
      "return_null_ptr", []() { return drake::EigenPtr<MatrixX<T>>(nullptr); });
}

}  // namespace pydrake
}  // namespace drake
