#include <map>
#include <sstream>

#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {

// TODO(eric.cousineau): Use py::self for operator overloads?
PYBIND11_MODULE(_symbolic_py, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::symbolic;

  using std::map;
  using std::ostringstream;

  m.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";

  py::class_<Variable>(m, "Variable")
      .def(py::init<const std::string&>())
      .def("get_id", &Variable::get_id)
      .def("__repr__", &Variable::to_string)
      .def("__hash__",
           [](const Variable& self) { return std::hash<Variable>{}(self); })
      .def("__add__",
           [](const Variable& self, const Variable& other) {
             return Expression{self + other};
           },
           py::is_operator())
      .def("__add__",
           [](const Variable& self, double other) {
             return Expression{self + other};
           },
           py::is_operator())
      .def("__add__",
           [](const Variable& self, const Expression& other) {
             return Expression{self + other};
           },
           py::is_operator())
      .def("__radd__",
           [](const Variable& self, double other) {
             return Expression{other + self};
           },
           py::is_operator())
      .def("__sub__",
           [](const Variable& self, const Variable& other) {
             return Expression{self - other};
           },
           py::is_operator())
      .def("__sub__",
           [](const Variable& self, double other) {
             return Expression{self - other};
           },
           py::is_operator())
      .def("__sub__",
           [](const Variable& self, const Expression& other) {
             return Expression{self - other};
           },
           py::is_operator())
      .def("__rsub__",
           [](const Variable& self, double other) {
             return Expression{other - self};
           },
           py::is_operator())
      .def("__mul__",
           [](const Variable& self, const Variable& other) {
             return Expression{self * other};
           },
           py::is_operator())
      .def("__mul__",
           [](const Variable& self, double other) {
             return Expression{self * other};
           },
           py::is_operator())
      .def("__mul__",
           [](const Variable& self, const Expression& other) {
             return Expression{self * other};
           },
           py::is_operator())
      .def("__mul__",
           [](const Variable& self, const Monomial& other) {
             return Monomial(self) * other;
           },
           py::is_operator())
      .def("__rmul__",
           [](const Variable& self, double other) {
             return Expression{other * self};
           },
           py::is_operator())
      .def("__truediv__",
           [](const Variable& self, const Variable& other) {
             return Expression{self / other};
           },
           py::is_operator())
      .def("__truediv__",
           [](const Variable& self, double other) {
             return Expression{self / other};
           },
           py::is_operator())
      .def("__truediv__",
           [](const Variable& self, const Expression& other) {
             return Expression{self / other};
           },
           py::is_operator())
      .def("__rtruediv__",
           [](const Variable& self, double other) {
             return Expression{other / self};
           },
           py::is_operator())
      .def("__pow__",
           [](const Variable& self, int other) {
             return Expression{pow(self, other)};
           },
           py::is_operator())
      .def("__pow__",
           [](const Variable& self, double other) {
             return Expression{pow(self, other)};
           },
           py::is_operator())
      .def("__pow__",
           [](const Variable& self, const Variable& other) {
             return Expression{pow(self, other)};
           },
           py::is_operator())
      .def("__pow__",
           [](const Variable& self, const Expression& other) {
             return Expression{pow(self, other)};
           },
           py::is_operator())
      .def("__neg__", [](const Variable& self) { return Expression{-self}; },
           py::is_operator())
      .def("__lt__",
           [](const Variable& self, const Variable& other) {
             return Formula{Expression(self) < Expression(other)};
           },
           py::is_operator())
      .def("__lt__",
           [](const Variable& self, double other) {
             return Formula{Expression(self) < Expression(other)};
           },
           py::is_operator())
      .def("__lt__",
           [](const Variable& self, const Expression& other) {
             return Formula{Expression(self) < other};
           },
           py::is_operator())
      .def("__le__",
           [](const Variable& self, const Variable& other) {
             return Formula{Expression(self) <= Expression(other)};
           },
           py::is_operator())
      .def("__le__",
           [](const Variable& self, double other) {
             return Formula{Expression(self) <= Expression(other)};
           },
           py::is_operator())
      .def("__le__",
           [](const Variable& self, const Expression& other) {
             return Formula{Expression(self) <= other};
           },
           py::is_operator())
      .def("__gt__",
           [](const Variable& self, const Variable& other) {
             return Formula{Expression(self) > Expression(other)};
           },
           py::is_operator())
      .def("__gt__",
           [](const Variable& self, double other) {
             return Formula{Expression(self) > Expression(other)};
           },
           py::is_operator())
      .def("__gt__",
           [](const Variable& self, const Expression& other) {
             return Formula{Expression(self) > other};
           },
           py::is_operator())
      .def("__ge__",
           [](const Variable& self, const Variable& other) {
             return Formula{Expression(self) >= Expression(other)};
           },
           py::is_operator())
      .def("__ge__",
           [](const Variable& self, double other) {
             return Formula{Expression(self) >= Expression(other)};
           },
           py::is_operator())
      .def("__ge__",
           [](const Variable& self, const Expression& other) {
             return Formula{Expression(self) >= other};
           },
           py::is_operator())
      .def("__eq__",
           [](const Variable& self, const Variable& other) {
             return Formula{Expression(self) == Expression(other)};
           },
           py::is_operator())
      .def("__eq__",
           [](const Variable& self, double other) {
             return Formula{Expression(self) == Expression(other)};
           },
           py::is_operator())
      .def("__eq__",
           [](const Variable& self, const Expression& other) {
             return Formula{Expression(self) == other};
           },
           py::is_operator());

  py::class_<Variables>(m, "Variables")
      .def(py::init<>())
      .def(py::init<const Eigen::Ref<const VectorX<Variable>>&>())
      .def("size", &Variables::size)
      .def("empty", &Variables::empty)
      .def("to_string", &Variables::to_string)
      .def("__hash__",
           [](const Variables& self) { return std::hash<Variables>{}(self); })
      .def("insert",
           [](Variables& self, const Variable& var) { self.insert(var); })
      .def("insert",
           [](Variables& self, const Variables& vars) { self.insert(vars); })
      .def("erase",
           [](Variables& self, const Variable& var) { return self.erase(var); })
      .def("erase", [](Variables& self,
                       const Variables& vars) { return self.erase(vars); })
      .def("include", &Variables::include)
      .def("IsSubsetOf", &Variables::IsSubsetOf)
      .def("IsSupersetOf", &Variables::IsSupersetOf)
      .def("IsStrictSubsetOf", &Variables::IsStrictSubsetOf)
      .def("IsStrictSupersetOf", &Variables::IsStrictSupersetOf)
      .def(py::self == py::self)
      .def(py::self < py::self)
      .def(py::self + py::self)
      .def(py::self + Variable())
      .def(Variable() + py::self)
      .def(py::self - py::self)
      .def(py::self - Variable());

  m.def("intersect", [](const Variables& vars1, const Variables& vars2) {
    return intersect(vars1, vars2);
  });

  py::class_<Expression>(m, "Expression")
      .def(py::init<>())
      .def(py::init<double>())
      .def(py::init<const Variable&>())
      .def("__repr__", &Expression::to_string)
      .def("Expand", &Expression::Expand)
      .def(py::self + py::self)
      .def(py::self + Variable())
      .def(py::self + double())
      .def(double() + py::self)
      .def(py::self - py::self)
      .def(py::self - Variable())
      .def(py::self - double())
      .def(double() - py::self)
      .def(py::self * py::self)
      .def(py::self * Variable())
      .def(py::self * double())
      .def(double() * py::self)
      .def(py::self / py::self)
      .def(py::self / Variable())
      .def(py::self / double())
      .def(double() / py::self)
      .def("__pow__",
           [](const Expression& self, int other) { return pow(self, other); },
           py::is_operator())
      .def(
          "__pow__",
          [](const Expression& self, double other) { return pow(self, other); },
          py::is_operator())
      .def("__pow__",
           [](const Expression& self, const Variable& other) {
             return pow(self, other);
           },
           py::is_operator())
      .def("__pow__",
           [](const Expression& self, const Expression& other) {
             return pow(self, other);
           },
           py::is_operator())
      .def("__neg__", [](const Expression& self) { return -self; },
           py::is_operator())
      .def("__lt__",
           [](const Expression& self, const Variable& other) {
             return Formula{Expression(self) < Expression(other)};
           },
           py::is_operator())
      .def("__lt__",
           [](const Expression& self, const Expression& other) {
             return Formula{Expression(self) < Expression(other)};
           },
           py::is_operator())
      .def("__lt__",
           [](const Expression& self, double other) {
             return Formula{Expression(self) < Expression(other)};
           },
           py::is_operator())
      .def("__le__",
           [](const Expression& self, const Variable& other) {
             return Formula{Expression(self) <= Expression(other)};
           },
           py::is_operator())
      .def("__le__",
           [](const Expression& self, const Expression& other) {
             return Formula{Expression(self) <= Expression(other)};
           },
           py::is_operator())
      .def("__le__",
           [](const Expression& self, double other) {
             return Formula{Expression(self) <= Expression(other)};
           },
           py::is_operator())
      .def("__gt__",
           [](const Expression& self, const Variable& other) {
             return Formula{Expression(self) > Expression(other)};
           },
           py::is_operator())
      .def("__gt__",
           [](const Expression& self, const Expression& other) {
             return Formula{Expression(self) > Expression(other)};
           },
           py::is_operator())
      .def("__gt__",
           [](const Expression& self, double other) {
             return Formula{Expression(self) > Expression(other)};
           },
           py::is_operator())
      .def("__ge__",
           [](const Expression& self, const Variable& other) {
             return Formula{Expression(self) >= Expression(other)};
           },
           py::is_operator())
      .def("__ge__",
           [](const Expression& self, const Expression& other) {
             return Formula{Expression(self) >= Expression(other)};
           },
           py::is_operator())
      .def("__ge__",
           [](const Expression& self, double other) {
             return Formula{Expression(self) >= Expression(other)};
           },
           py::is_operator())
      .def("__eq__",
           [](const Expression& self, const Variable& other) {
             return Formula{Expression(self) == Expression(other)};
           },
           py::is_operator())
      .def("__eq__",
           [](const Expression& self, const Expression& other) {
             return Formula{Expression(self) == Expression(other)};
           },
           py::is_operator())
      .def("__eq__",
           [](const Expression& self, double other) {
             return Formula{Expression(self) == Expression(other)};
           },
           py::is_operator());

  py::class_<Formula>(m, "Formula").def("__repr__", &Formula::to_string);

  // Cannot overload logical operators: http://stackoverflow.com/a/471561
  // Defining custom function for clarity.
  // Could use bitwise operators:
  // https://docs.python.org/2/library/operator.html#operator.__and__
  // However, this may reduce clarity and introduces constraints on order of
  // operations.
  m
      // Hide AND and OR to permit us to make it accept 1 or more arguments in
      // Python (and not have to handle type safety within C++).
      .def("__logical_and",
           [](const Formula& a, const Formula& b) { return a && b; })
      .def("__logical_or",
           [](const Formula& a, const Formula& b) { return a || b; })
      .def("logical_not", [](const Formula& a) { return !a; });

  py::class_<Monomial>(m, "Monomial")
      .def(py::init<const Variable&>())
      .def(py::init<const Variable&, int>())
      .def(py::init<const map<Variable, int>&>())
      .def("degree", &Monomial::degree)
      .def("total_degree", &Monomial::total_degree)
      .def(py::self * py::self)
      .def("__mul__",
           [](const Monomial& self, const Variable& other) {
             return self * Monomial(other);
           },
           py::is_operator())
      .def(py::self == py::self)
      .def("__hash__",
           [](const Monomial& self) { return std::hash<Monomial>{}(self); })
      .def(py::self != py::self)
      .def("__repr__",
           [](const Monomial& self) {
             ostringstream oss;
             oss << self;
             return oss.str();
           })
      .def("GetVariables", &Monomial::GetVariables)
      .def("get_powers", &Monomial::get_powers, py_reference_internal)
      .def("ToExpression", &Monomial::ToExpression)
      .def("pow_in_place", &Monomial::pow_in_place, py_reference_internal)
      .def("__pow__",
           [](const Monomial& self, const int p) { return pow(self, p); });

  m.def("MonomialBasis",
        [](const Eigen::Ref<const VectorX<Variable>>& vars, const int degree) {
          return MonomialBasis(Variables{vars}, degree);
        })
      .def("MonomialBasis", [](const Variables& vars, const int degree) {
        return MonomialBasis(vars, degree);
      });

  py::class_<Polynomial>(m, "Polynomial")
      .def(py::init<>())
      .def(py::init<Polynomial::MapType>())
      .def(py::init<const Monomial&>())
      .def(py::init<const Expression&>())
      .def(py::init<const Expression&, const Variables&>())
      .def(py::init([](const Expression& e,
                       const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Polynomial{e, Variables{vars}};
      }))
      .def("indeterminates", &Polynomial::indeterminates)
      .def("decision_variables", &Polynomial::decision_variables)
      .def("Degree", &Polynomial::Degree)
      .def("TotalDegree", &Polynomial::TotalDegree)
      .def("monomial_to_coefficient_map",
           &Polynomial::monomial_to_coefficient_map)
      .def("ToExpression", &Polynomial::ToExpression)
      .def("Differentiate", &Polynomial::Differentiate)
      .def("AddProduct", &Polynomial::AddProduct)
      .def(py::self + py::self)
      .def(py::self + Monomial())
      .def(Monomial() + py::self)
      .def(py::self + double())
      .def(double() + py::self)
      .def(py::self - py::self)
      .def(py::self - Monomial())
      .def(Monomial() - py::self)
      .def(py::self - double())
      .def(double() - py::self)
      .def(py::self * py::self)
      .def(py::self * Monomial())
      .def(Monomial() * py::self)
      .def(py::self * double())
      .def(double() * py::self)
      .def(-py::self)
      .def("EqualTo", &Polynomial::EqualTo)
      .def(py::self == py::self)
      .def("__repr__",
           [](const Polynomial& self) {
             ostringstream oss;
             oss << self;
             return oss.str();
           })
      .def("__pow__",
           [](const Polynomial& self, const int n) { return pow(self, n); });
}

}  // namespace pydrake
}  // namespace drake
