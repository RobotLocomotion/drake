#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "autodiff_types.h"


namespace py = pybind11;

PYBIND11_PLUGIN(_autodiffutils) {
  py::module m("_autodiffutils", "Bindings for Eigen AutoDiff Scalars");

  py::class_<VectorXAutoDiffXd>(m, "VectorXAutoDiffXd")
    .def("__init__",
         [](VectorXAutoDiffXd& self, const Eigen::VectorXd& values, const Eigen::MatrixXd& derivatives) {
            new (&self) VectorXAutoDiffXd(values.size());
            if (derivatives.rows() != values.rows()) {
              throw std::runtime_error("derivatives must have exactly one row for every element in values");
            }
            for (Eigen::Index i=0; i < values.size(); i++) {
              self.coeffRef(i) = Eigen::AutoDiffScalar<Eigen::VectorXd>(values(i), derivatives.row(i));
            }
          })
    .def("derivatives",
         [](const VectorXAutoDiffXd& self) {
          Eigen::MatrixXd result(self.size(), self.size() > 0 ? self.coeffRef(0).derivatives().size() : 0);
          for (Eigen::Index i=0; i < self.size(); i++) {
            if (self.coeffRef(i).derivatives().size() > result.cols()) {
              result.conservativeResize(result.rows(), self.coeffRef(i).derivatives().size());
            }
            for (int j=0; j < self.coeffRef(i).derivatives().size(); j++) {
              result(i, j) = self.coeffRef(i).derivatives()(j);
            }
          }
          return result;
        })
    .def("value",
         [](const VectorXAutoDiffXd& self) {
            Eigen::VectorXd result(self.size());
            for (Eigen::Index i=0; i < self.size(); i++) {
              result(i) = self.coeffRef(i).value();
            }
            return result;
          });

  py::class_<Matrix3XAutoDiffXd>(m, "Matrix3XAutoDiffXd")
    .def("derivatives",
         [](const Matrix3XAutoDiffXd& self) {
          Eigen::MatrixXd result(self.size(), self.size() > 0 ? self.coeffRef(0).derivatives().size() : 0);
          for (Eigen::Index i=0; i < self.size(); i++) {
            if (self.coeffRef(i).derivatives().size() > result.cols()) {
              result.conservativeResize(result.rows(), self.coeffRef(i).derivatives().size());
            }
            for (int j=0; j < self.coeffRef(i).derivatives().size(); j++) {
              result(i, j) = self.coeffRef(i).derivatives()(j);
            }
          }
          return result;
        })
    .def("value",
         [](const Matrix3XAutoDiffXd& self) {
            Eigen::VectorXd result(self.size());
            for (Eigen::Index i=0; i < self.size(); i++) {
              result(i) = self.coeffRef(i).value();
            }
            return result;
          });


  return m.ptr();
}