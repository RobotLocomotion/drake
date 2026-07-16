#include "nanobind/nanobind.h"

#include "drake_nanobind/eigen/dense.h"
#include "drake_nanobind/eigen/sparse.h"

namespace py = nanobind;

NB_MODULE(sample_module, m) {
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
