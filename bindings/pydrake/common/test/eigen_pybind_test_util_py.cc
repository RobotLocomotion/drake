#include "drake/bindings/pydrake/common/eigen_pybind.h"

namespace drake {
namespace pydrake {

class MyObject {
 public:
  MyObject() {}

  Eigen::VectorXd PassThrough(const Eigen::VectorXd& x) { return x; }

  // Can think of this as a function that will change the input dimension.
  Eigen::VectorXd ReturnOnes(const Eigen::VectorXd&) {
    return Eigen::VectorXd::Ones(2);
  }
};

PYBIND11_MODULE(eigen_pybind_test_util, m) {
  m.doc() = "Example bindings that use drake::EigenPtr types, for testing.";
  py::module::import("pydrake.common");

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

  {
    using Class = MyObject;
    py::class_<Class> cls(m, "MyObject");
    cls  // BR
        .def(py::init())
        .def("PassThroughNoWrap", &Class::PassThrough)
        .def("PassThroughWithWrap", &Class::PassThrough)
        .def("ReturnOnesNoWrap", &Class::ReturnOnes)
        .def("ReturnOnesWithWrap", &Class::ReturnOnes);
    cls.attr("PassThroughWithWrap") = WrapToMatchInputShape(
        cls.attr("PassThroughWithWrap"));
    cls.attr("ReturnOnesWithWrap") = WrapToMatchInputDimension(
        cls.attr("ReturnOnesWithWrap"));
  }
}

}  // namespace pydrake
}  // namespace drake
