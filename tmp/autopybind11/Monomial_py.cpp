#include "drake/common/symbolic.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py = pybind11;
void apb11_pydrake_Monomial_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::symbolic;

  py::class_<Monomial> PyMonomial(
      m, "Monomial",
      R"""(/** Represents a monomial, a product of powers of variables with non-negative 
 * integer exponents. Note that it does not include the coefficient part of a 
 * monomial. 
 */)""");

  PyMonomial.def(py::init<Monomial const &>(),py::arg("arg0"))
    .def(py::init<>())
    .def(py::init<::std::nullptr_t>(),py::arg("arg0"))
    .def(py::init<::std::map<drake::symbolic::Variable, int, std::less<drake::symbolic::Variable>, std::allocator<std::pair<const drake::symbolic::Variable, int>>> const &>(),py::arg("powers"))
    .def(py::init<::Eigen::Ref<const Eigen::Matrix<drake::symbolic::Variable, -1, 1, 0, -1, 1>, 0, Eigen::InnerStride<1>> const &,::Eigen::Ref<const Eigen::Matrix<int, -1, 1, 0, -1, 1>, 0, Eigen::InnerStride<1>> const &>(),py::arg("vars"),py::arg("exponents"))
    .def(py::init<Expression const &>(),py::arg("e"))
    .def(py::init<Variable const &>(),py::arg("var"))
    .def(py::init<Variable const &,int>(),py::arg("var"),py::arg("exponent"))
    .def_static("DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE", static_cast<void (*)(  )>(&Monomial::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
    .def("Evaluate", static_cast<double ( Monomial::* )( Environment const & )const>(&Monomial::Evaluate), py::arg("env"), 
R"""(/** Evaluates under a given environment @p env. 
 * 
 * @throws std::out_of_range exception if there is a variable in this monomial 
 * whose assignment is not provided by @p env. 
 */)""")
    .def("EvaluatePartial", static_cast<::std::pair<double, drake::symbolic::Monomial> ( Monomial::* )( Environment const & )const>(&Monomial::EvaluatePartial), py::arg("env"), 
R"""(/** Partially evaluates using a given environment @p env. The evaluation 
 * result is of type pair<double, Monomial>. The first component (: double) 
 * represents the coefficient part while the second component represents the 
 * remaining parts of the Monomial which was not evaluated. 
 * 
 * Example 1. Evaluate with a fully-specified environment 
 *     (x³*y²).EvaluatePartial({{x, 2}, {y, 3}}) 
 *   = (2³ * 3² = 8 * 9 = 72, Monomial{} = 1). 
 * 
 * Example 2. Evaluate with a partial environment 
 *     (x³*y²).EvaluatePartial({{x, 2}}) 
 *   = (2³ = 8, y²). 
 */)""")
    .def("GetVariables", static_cast<Variables ( Monomial::* )(  )const>(&Monomial::GetVariables), 
R"""(/** Returns the set of variables in this monomial. */)""")
    .def("ToExpression", static_cast<Expression ( Monomial::* )(  )const>(&Monomial::ToExpression), 
R"""(/** Returns a symbolic expression representing this monomial. */)""")
    .def("degree", static_cast<int ( Monomial::* )( Variable const & )const>(&Monomial::degree), py::arg("v"), 
R"""(/** Returns the degree of this Monomial in a variable @p v. */)""")
    .def("get_powers", static_cast<::std::map<drake::symbolic::Variable, int, std::less<drake::symbolic::Variable>, std::allocator<std::pair<const drake::symbolic::Variable, int>>> const & ( Monomial::* )(  )const>(&Monomial::get_powers), 
R"""(/** Returns the internal representation of Monomial, the map from a base 
 * (Variable) to its exponent (int).*/)""")
    .def("pow_in_place", static_cast<Monomial & ( Monomial::* )( int )>(&Monomial::pow_in_place), py::arg("p"), 
R"""(/** Returns this monomial raised to @p p. 
 * @throws std::runtime_error if @p p is negative. 
 */)""")
    .def("total_degree", static_cast<int ( Monomial::* )(  )const>(&Monomial::total_degree), 
R"""(/** Returns the total degree of this Monomial. */)""")
    
    .def("__neq__", static_cast<bool ( Monomial::* )( Monomial const & )const>(&Monomial::operator!=), py::arg("m"), 
R"""(/** Checks if this monomial and @p m do not represent the same monomial. */)""")
    .def("__imul__", static_cast<Monomial & ( Monomial::* )( Monomial const & )>(&Monomial::operator*=), py::arg("m"), 
R"""(/** Returns this monomial multiplied by @p m. */)""")
    .def("__str__", +[](Monomial const & m) {
        std::ostringstream oss;
        oss << m;
        std::string s(oss.str());

        return s;})        .def("__eq__", static_cast<bool ( Monomial::* )( Monomial const & )const>(&Monomial::operator==), py::arg("m"), 
R"""(/** Checks if this monomial and @p m represent the same monomial. Two 
 * monomials are equal iff they contain the same variable raised to the same 
 * exponent. */)""")
    ;
}
