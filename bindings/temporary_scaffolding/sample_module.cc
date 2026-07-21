#include "nanobind/nanobind.h"

#include "drake_nanobind/eigen/dense.h"
#include "drake_nanobind/eigen/sparse.h"

namespace py = nanobind;

NB_MODULE(sample_module, m) {
  m  // BR
      .def("copy_dense_mat",
           [](const Eigen::MatrixXd& x) -> Eigen::MatrixXd {
             return x;
           })
      .def("copy_dense_mat_ref",
           [](const Eigen::Ref<const Eigen::MatrixXd>& x) -> Eigen::MatrixXd {
             return x;
           })
      .def("copy_dense_vec",
           [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
             return x;
           })
      .def("copy_dense_vec_ref",
           [](const Eigen::Ref<const Eigen::VectorXd>& x) -> Eigen::VectorXd {
             return x;
           })
      .def("dense_to_sparse",
           [](const Eigen::MatrixXd& x) -> Eigen::SparseMatrix<double> {
             return x.sparseView();
           })
      .def("sparse_to_dense",
           [](const Eigen::SparseMatrix<double>& x) -> Eigen::MatrixXd {
             return x;
           });
}
