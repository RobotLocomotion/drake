#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(polynomial, m) {
  {
    py::module::import("pydrake.common");

    using T = double;
    using Class = Polynomial<T>;
    constexpr auto& cls_doc = pydrake_doc.drake.Polynomial;
    py::class_<Class>(m, "Polynomial", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<const T&>(), cls_doc.ctor.doc_1args_scalar)
        .def(py::init<const Eigen::Ref<const Eigen::VectorXd>&>(),
            cls_doc.ctor.doc_1args_constEigenMatrixBase)
        .def("GetNumberOfCoefficients", &Class::GetNumberOfCoefficients,
            cls_doc.GetNumberOfCoefficients.doc)
        .def("GetDegree", &Class::GetDegree, cls_doc.GetDegree.doc)
        .def("IsAffine", &Class::IsAffine, cls_doc.IsAffine.doc)
        .def("GetCoefficients", &Class::GetCoefficients,
            cls_doc.GetCoefficients.doc)
        .def("Derivative", &Class::Derivative, py::arg("derivative_order") = 1,
            cls_doc.Derivative.doc)
        .def("Integral", &Class::Integral,
            py::arg("integration_constant") = 0.0, cls_doc.Integral.doc)
        .def("CoefficientsAlmostEqual", &Class::CoefficientsAlmostEqual,
            py::arg("other"), py::arg("tol") = 0.0,
            py::arg("tol_type") = ToleranceType::kAbsolute,
            cls_doc.CoefficientsAlmostEqual.doc)
        // Arithmetic
        .def(-py::self)
        .def(py::self + py::self)
        .def(py::self + double())
        .def(double() + py::self)
        .def(py::self - py::self)
        .def(py::self - double())
        .def(double() - py::self)
        .def(py::self * py::self)
        .def(py::self * double())
        .def(double() * py::self)
        .def(py::self / double())
        // Logical comparison
        .def(py::self == py::self);
  }
}

}  // namespace pydrake
}  // namespace drake
