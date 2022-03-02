#include "drake/bindings/pydrake/common/eigen_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(eigen_pybind_test_util, m) {
  m.doc() = "Example bindings that use drake::EigenPtr types, for testing.";

  using T = double;

  m.def("takes_returns_matrix_pointer",
      [](drake::EigenPtr<MatrixX<T>> mat) { return mat; });

  m.def("scale_matrix_ptr", [](drake::EigenPtr<MatrixX<T>> mat, T factor) {
    if (mat != nullptr) {
      *mat *= factor;
    }
  });

  m.def(
      "return_null_ptr", []() { return drake::EigenPtr<MatrixX<T>>(nullptr); });
}

}  // namespace pydrake
}  // namespace drake
