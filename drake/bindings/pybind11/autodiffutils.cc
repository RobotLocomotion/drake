#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "drake/bindings/pybind11/autodiff_types.h"


namespace py = pybind11;

using std::sin;
using std::cos;

template <typename Derived>
AutoDiffXd eval(const Eigen::AutoDiffScalar<Derived>& x) {
  return AutoDiffXd(x.value(), x.derivatives());
}

PYBIND11_PLUGIN(_autodiffutils) {
  py::module m("_autodiffutils", "Bindings for Eigen AutoDiff Scalars");

  py::class_<AutoDiffXd>(m, "AutoDiffXd")
    .def("__init__",
         [](AutoDiffXd& self,
            double value,
            const Eigen::VectorXd& derivatives) {
           new (&self) AutoDiffXd(value, derivatives);
         })
    .def("value", [](const AutoDiffXd& self) {
      return self.value();
    })
    .def("derivatives", [](const AutoDiffXd& self) {
      return self.derivatives();
    })
    .def("sin", [](const AutoDiffXd& self) { return eval(sin(self)); })
    .def("cos", [](const AutoDiffXd& self) { return eval(cos(self)); })
    .def("__add__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self + other);
    })
    .def("__add__", [](const AutoDiffXd& self, double other) {
      return eval(self + other);
    })
    .def("__radd__", [](const AutoDiffXd& self, double other) {
      return eval(other + self);
    })
    .def("__sub__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self - other);
    })
    .def("__sub__", [](const AutoDiffXd& self, double other) {
      return eval(self - other);
    })
    .def("__rsub__", [](const AutoDiffXd& self, double other) {
      return eval(other - self);
    })
    .def("__mul__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self * other);
    })
    .def("__mul__", [](const AutoDiffXd& self, double other) {
      return eval(self * other);
    })
    .def("__rmul__", [](const AutoDiffXd& self, double other) {
      return eval(other * self);
    })
    .def("__truediv__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self / other);
    })
    .def("__truediv__", [](const AutoDiffXd& self, double other) {
      return eval(self / other);
    })
    .def("__rtruediv__", [](const AutoDiffXd& self, double other) {
      return eval(other / self);
    });

  py::class_<VectorXAutoDiffXd>(m, "VectorXAutoDiffXd")
    .def("__init__",
         [](VectorXAutoDiffXd& self, std::vector<int> shape) {
          if (shape.size() == 1) {
            new (&self) VectorXAutoDiffXd(shape[0]);
          } else {
            throw std::runtime_error(
          "VectorXAutoDiffXd must be initialized with a one-dimensional shape");
          }
        })
    .def("shape",
         [](const VectorXAutoDiffXd& self) {
      std::vector<Eigen::Index> shape = {self.size()};
      return shape;
    })
    .def("size",
         [](const VectorXAutoDiffXd& self) {
      return self.size();
    })
    .def("__getitem__",
         [](const VectorXAutoDiffXd& self, size_t i) {
      return self(i);
    })
    .def("__setitem__",
         [](VectorXAutoDiffXd& self, size_t i, const AutoDiffXd& value) {
      self(i) = value;
    });

  py::class_<Matrix3XAutoDiffXd>(m, "Matrix3XAutoDiffXd")
    .def("__init__",
         [](Matrix3XAutoDiffXd& self, std::vector<int> shape) {
          if (shape.size() != 2) {
            throw std::runtime_error(
        "Matrix3XAutoDiffXd must be initialized with a two-dimensional shape");
          }
          new (&self) Matrix3XAutoDiffXd(shape[0], shape[1]);
        })
    .def("shape",
         [](const Matrix3XAutoDiffXd& self) {
      std::vector<Eigen::Index> shape = {self.rows(), self.cols()};
      return shape;
    })
    .def("size",
         [](const Matrix3XAutoDiffXd& self) {
      return self.size();
    })
    .def("__getitem__",
         [](const Matrix3XAutoDiffXd& self, size_t i) {
      return self(i);
    })
    .def("__setitem__",
         [](Matrix3XAutoDiffXd& self, size_t i, const AutoDiffXd& value) {
      self(i) = value;
    });

  return m.ptr();
}
