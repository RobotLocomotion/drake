#include "drake/common/symbolic.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

using namespace drake::symbolic;

namespace py = pybind11;
void apb11_pydrake_Monomial_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<Monomial> PyMonomial(m, "Monomial");

  PyMonomial.def(py::init<Monomial const &>(),py::arg("arg0"))
    .def(py::init<>())
    .def(py::init<::std::nullptr_t>(),py::arg("arg0"))
    .def(py::init<::std::map<Variable, int, std::less<Variable>, std::allocator<std::pair<const Variable, int>>> const &>(),py::arg("powers"))
    .def(py::init<::Eigen::Ref<const Eigen::Matrix<Variable, -1, 1, 0, -1, 1>, 0, Eigen::InnerStride<1>> const &,::Eigen::Ref<const Eigen::Matrix<int, -1, 1, 0, -1, 1>, 0, Eigen::InnerStride<1>> const &>(),py::arg("vars"),py::arg("exponents"))
    .def(py::init<Expression const &>(),py::arg("e"))
    .def(py::init<Variable const &>(),py::arg("var"))
    .def(py::init<Variable const &,int>(),py::arg("var"),py::arg("exponent"))
    .def("Evaluate", static_cast<double ( Monomial::* )( Environment const & )const>(&Monomial::Evaluate), py::arg("env"))
    .def("EvaluatePartial", static_cast<::std::pair<double, Monomial> ( Monomial::* )( Environment const & )const>(&Monomial::EvaluatePartial), py::arg("env"))
    .def("GetVariables", static_cast<Variables ( Monomial::* )(  )const>(&Monomial::GetVariables))
    .def("ToExpression", static_cast<Expression ( Monomial::* )(  )const>(&Monomial::ToExpression))
    .def("degree", static_cast<int ( Monomial::* )( Variable const & )const>(&Monomial::degree), py::arg("v"))
    .def("get_powers", static_cast<::std::map<Variable, int, std::less<Variable>, std::allocator<std::pair<const Variable, int>>> const & ( Monomial::* )(  )const>(&Monomial::get_powers))
    .def("pow_in_place", static_cast<Monomial & ( Monomial::* )( int )>(&Monomial::pow_in_place), py::arg("p"))
    .def("total_degree", static_cast<int ( Monomial::* )(  )const>(&Monomial::total_degree))
    
    .def("__neq__", static_cast<bool ( Monomial::* )( Monomial const & )const>(&Monomial::operator!=), py::arg("m"))
    .def("__imul__", static_cast<Monomial & ( Monomial::* )( Monomial const & )>(&Monomial::operator*=), py::arg("m"))
    .def("__str__", +[](Monomial const & m) {
        std::ostringstream oss;
        oss << m;
        std::string s(oss.str());

        return s;})        .def("__eq__", static_cast<bool ( Monomial::* )( Monomial const & )const>(&Monomial::operator==), py::arg("m"))
    ;
}
