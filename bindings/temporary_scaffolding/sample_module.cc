// clang-format off
#if defined(DRAKE_USE_PYBIND11)
# include "pybind11/pybind11.h"
# include "pybind11/eigen.h"
namespace py = pybind11;
#define DEFINE_MODULE PYBIND11_MODULE
#elif defined(DRAKE_USE_NANOBIND)
# include "nanobind/nanobind.h"
# include "drake_nanobind/eigen/dense.h"
# include "drake_nanobind/eigen/sparse.h"
namespace py = nanobind;
#define DEFINE_MODULE NB_MODULE
#else
# error
#endif
// clang-format on

DEFINE_MODULE(sample_module, m) {
  m  // BR
      .def("dense_to_sparse",
           [](const Eigen::MatrixXd& x) -> Eigen::SparseMatrix<double> {
             return x.sparseView();
           })
      .def("sparse_to_dense",
           [](const Eigen::SparseMatrix<double>& x) -> Eigen::MatrixXd {
             return x;
           });
}
