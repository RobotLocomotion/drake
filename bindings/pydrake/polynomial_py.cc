#include <utility>
#include <vector>

#include "drake/bindings/generated_docstrings/common.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/polynomial_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/polynomial.h"

namespace drake {
namespace pydrake {

namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  using Class = Polynomial<T>;
  constexpr auto& cls_doc = pydrake_doc_common.drake.Polynomial;
  auto cls = DefineTemplateClassWithDefault<Class>(
      m, "Polynomial", param, cls_doc.doc);
  cls  // BR
      .def(py::init<>(), cls_doc.ctor.doc_0args)
      .def(py::init<const T&>(), cls_doc.ctor.doc_1args_scalar)
      .def(py::init<const Eigen::Ref<const Eigen::VectorXd>&>(),
          py::arg("coefficients"), cls_doc.ctor.doc_1args_constEigenMatrixBase)
      .def("GetNumberOfCoefficients", &Class::GetNumberOfCoefficients,
          cls_doc.GetNumberOfCoefficients.doc)
      .def("GetDegree", &Class::GetDegree, cls_doc.GetDegree.doc)
      .def("IsAffine", &Class::IsAffine, cls_doc.IsAffine.doc)
      .def("GetCoefficients", &Class::GetCoefficients,
          cls_doc.GetCoefficients.doc)
      .def(
          "EvaluateUnivariate",
          [](const Class* self, const T& x, int derivative_order) {
            return self->EvaluateUnivariate(x, derivative_order);
          },
          py::arg("x"), py::arg("derivative_order") = 0,
          cls_doc.EvaluateUnivariate.doc)
      .def("Derivative", &Class::Derivative, py::arg("derivative_order") = 1,
          cls_doc.Derivative.doc)
      .def("Integral", &Class::Integral, py::arg("integration_constant") = 0.0,
          cls_doc.Integral.doc)
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

  typedef std::vector<T> CoefficientsType;
  typedef std::vector<std::vector<
      std::pair<typename Class::VarType, typename Class::PowerType>>>
      TermsType;
  cls.def(py::pickle(
      [](const Class& self) {
        std::vector<typename Class::Monomial> monomials = self.GetMonomials();
        CoefficientsType coefficients;
        TermsType terms;
        for (const auto& monomial : monomials) {
          coefficients.emplace_back(monomial.coefficient);
          terms.emplace_back();
          for (const auto& term : monomial.terms) {
            terms.back().emplace_back(term.var, term.power);
          }
        }
        return std::make_pair(coefficients, terms);
      },
      [](std::pair<CoefficientsType, TermsType> args) {
        const CoefficientsType& coefficients = std::get<0>(args);
        const TermsType& terms_data = std::get<1>(args);
        DRAKE_DEMAND(coefficients.size() == terms_data.size());

        std::vector<typename Class::Monomial> monomials;
        for (int i = 0; i < static_cast<int>(coefficients.size()); ++i) {
          std::vector<typename Class::Term> monomial_terms;
          for (const auto& [var, power] : terms_data[i]) {
            monomial_terms.emplace_back(var, power);
          }
          monomials.emplace_back(coefficients[i], monomial_terms);
        }
        return Class(
            monomials.begin(), monomials.end(), /* canonicalize= */ false);
      }));
}
}  // namespace

PYBIND11_MODULE(polynomial, m) {
  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common");
  py::module::import("pydrake.symbolic");

  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
