#include <map>
#include <string>

#include "fmt/format.h"
#include "fmt/ostream.h"
#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

namespace drake {
namespace pydrake {

using std::map;
using std::string;

// TODO(eric.cousineau): Use py::self for operator overloads?
PYBIND11_MODULE(_symbolic_py, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::symbolic;

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should agressively try to avoid these warnings.
  py::module::import("pydrake.util.deprecation")
      .attr("install_numpy_warning_filters")();

  // Install NumPy formatters patch.
  py::module::import("pydrake.util.compatibility")
      .attr("maybe_patch_numpy_formatters")();

  m.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";

  py::class_<Variable>(m, "Variable")
      .def(py::init<const string&>())
      .def("get_id", &Variable::get_id)
      .def("__str__", &Variable::to_string)
      .def("__repr__",
           [](const Variable& self) {
             return fmt::format("Variable('{}')", self.to_string());
           })
      .def("__hash__",
           [](const Variable& self) { return std::hash<Variable>{}(self); })
      .def("__copy__", [](const Variable& self) -> Variable { return self; })
      // Addition.
      .def(py::self + py::self)
      .def(py::self + double())
      .def(double() + py::self)
      // Subtraction.
      .def(py::self - py::self)
      .def(py::self - double())
      .def(double() - py::self)
      // Multiplication.
      .def(py::self * py::self)
      .def(py::self * double())
      .def(double() * py::self)
      // Division.
      .def(py::self / py::self)
      .def(py::self / double())
      .def(double() / py::self)
      // Pow.
      .def("__pow__",
           [](const Variable& self, double other) { return pow(self, other); },
           py::is_operator())
      .def("__pow__",
           [](const Variable& self, const Variable& other) {
             return pow(self, other);
           },
           py::is_operator())
      .def("__pow__",
           [](const Variable& self, const Expression& other) {
             return pow(self, other);
           },
           py::is_operator())
      // We add `EqualTo` instead of `equal_to` to maintain consistency among
      // symbolic classes (Variable, Expression, Formula, Polynomial) on Python
      // side. This enables us to achieve polymorphism via ducktyping in Python.
      .def("EqualTo", &Variable::equal_to)
      // Unary Plus.
      .def(+py::self)
      // Unary Minus.
      .def(-py::self)
      // LT(<).
      // Note that for `double < Variable` case, the reflected op ('>' in this
      // case) is called. For example, `1 < x` will return `x > 1`.
      .def(py::self < Expression())
      .def(py::self < py::self)
      .def(py::self < double())
      // LE(<=).
      .def(py::self <= Expression())
      .def(py::self <= py::self)
      .def(py::self <= double())
      // GT(>).
      .def(py::self > Expression())
      .def(py::self > py::self)
      .def(py::self > double())
      // GE(>=).
      .def(py::self >= Expression())
      .def(py::self >= py::self)
      .def(py::self >= double())
      // EQ(==).
      .def(py::self == Expression())
      .def(py::self == py::self)
      .def(py::self == double())
      // NE(!=).
      .def(py::self != Expression())
      .def(py::self != py::self)
      .def(py::self != double());

  py::class_<Variables>(m, "Variables")
      .def(py::init<>())
      .def(py::init<const Eigen::Ref<const VectorX<Variable>>&>())
      .def("size", &Variables::size)
      .def("__len__", &Variables::size)
      .def("empty", &Variables::empty)
      .def("__str__", &Variables::to_string)
      .def("__repr__",
           [](const Variables& self) {
             return fmt::format("<Variables \"{}\">", self);
           })
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
      .def("__contains__", &Variables::include)
      .def("IsSubsetOf", &Variables::IsSubsetOf)
      .def("IsSupersetOf", &Variables::IsSupersetOf)
      .def("IsStrictSubsetOf", &Variables::IsStrictSubsetOf)
      .def("IsStrictSupersetOf", &Variables::IsStrictSupersetOf)
      .def("EqualTo", [](const Variables& self,
                         const Variables& vars) { return self == vars; })
      .def("__iter__",
           [](const Variables& vars) {
             return py::make_iterator(vars.begin(), vars.end());
           },
           // Keep alive, reference: `return` keeps `self` alive
           py::keep_alive<0, 1>())
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
      .def("__str__", &Expression::to_string)
      .def("__repr__",
           [](const Expression& self) {
             return fmt::format("<Expression \"{}\">", self.to_string());
           })
      .def("__copy__",
           [](const Expression& self) -> Expression { return self; })
      .def("to_string", &Expression::to_string)
      .def("Expand", &Expression::Expand)
      .def("Evaluate", [](const Expression& self) { return self.Evaluate(); })
      .def("Evaluate",
           [](const Expression& self, const Environment::map& env) {
             return self.Evaluate(Environment{env});
           })
      .def("EvaluatePartial",
           [](const Expression& self, const Environment::map& env) {
             return self.EvaluatePartial(Environment{env});
           })
      .def("Substitute",
           [](const Expression& self, const Variable& var,
              const Expression& e) { return self.Substitute(var, e); })
      .def("Substitute",
           [](const Expression& self, const Substitution& s) {
             return self.Substitute(s);
           })
      .def("EqualTo", &Expression::EqualTo)
      // Addition
      .def(py::self + py::self)
      .def(py::self + Variable())
      .def(py::self + double())
      .def(Variable() + py::self)
      .def(double() + py::self)
      .def(py::self += py::self)
      .def(py::self += Variable())
      .def(py::self += double())
      // Subtraction.
      .def(py::self - py::self)
      .def(py::self - Variable())
      .def(py::self - double())
      .def(Variable() - py::self)
      .def(double() - py::self)
      .def(py::self -= py::self)
      .def(py::self -= Variable())
      .def(py::self -= double())
      // Multiplication.
      .def(py::self * py::self)
      .def(py::self * Variable())
      .def(py::self * double())
      .def(Variable() * py::self)
      .def(double() * py::self)
      .def(py::self *= py::self)
      .def(py::self *= Variable())
      .def(py::self *= double())
      // Division.
      .def(py::self / py::self)
      .def(py::self / Variable())
      .def(py::self / double())
      .def(Variable() / py::self)
      .def(double() / py::self)
      .def(py::self /= py::self)
      .def(py::self /= Variable())
      .def(py::self /= double())
      // Pow.
      .def("__pow__", [](const Expression& self,
                         const double other) { return pow(self, other); })
      .def("__pow__", [](const Expression& self,
                         const Variable& other) { return pow(self, other); })
      .def("__pow__", [](const Expression& self,
                         const Expression& other) { return pow(self, other); })
      // Unary Plus.
      .def(+py::self)
      // Unary Minus.
      .def(-py::self)
      // LT(<).
      //
      // Note that for `double < Expression` case, the reflected op ('>' in this
      // case) is called. For example, `1 < x * y` will return `x * y > 1`.
      .def(py::self < py::self)
      .def(py::self < Variable())
      .def(py::self < double())
      // LE(<=).
      .def(py::self <= py::self)
      .def(py::self <= Variable())
      .def(py::self <= double())
      // GT(>).
      .def(py::self > py::self)
      .def(py::self > Variable())
      .def(py::self > double())
      // GE(>=).
      .def(py::self >= py::self)
      .def(py::self >= Variable())
      .def(py::self >= double())
      // EQ(==).
      .def(py::self == py::self)
      .def(py::self == Variable())
      .def(py::self == double())
      // NE(!=)
      .def(py::self != py::self)
      .def(py::self != Variable())
      .def(py::self != double())
      .def("Differentiate", &Expression::Differentiate)
      .def("Jacobian", &Expression::Jacobian)
      // TODO(eric.cousineau): Figure out how to consolidate with the below
      // methods.
      .def("log", &symbolic::log)
      .def("__abs__", &symbolic::abs)
      .def("exp", &symbolic::exp)
      .def("sqrt", &symbolic::sqrt)
      // TODO(eric.cousineau): Move `__pow__` here.
      .def("sin", &symbolic::sin)
      .def("cos", &symbolic::cos)
      .def("tan", &symbolic::tan)
      .def("arcsin", &symbolic::asin)
      .def("arccos", &symbolic::acos)
      .def("arctan2", &symbolic::atan2)
      .def("sinh", &symbolic::sinh)
      .def("cosh", &symbolic::cosh)
      .def("tanh", &symbolic::tanh)
      .def("min", &symbolic::min)
      .def("max", &symbolic::max)
      .def("ceil", &symbolic::ceil)
      .def("floor", &symbolic::floor);

  // TODO(eric.cousineau): Consider deprecating these methods?
  auto math = py::module::import("pydrake.math");
  MirrorDef<py::module, py::module>(&math, &m)
      .def("log", &symbolic::log)
      .def("abs", &symbolic::abs)
      .def("exp", &symbolic::exp)
      .def("sqrt", &symbolic::sqrt)
      .def("pow", py::overload_cast<const Expression&, const Expression&>(
                      &symbolic::pow))
      .def("sin", &symbolic::sin)
      .def("cos", &symbolic::cos)
      .def("tan", &symbolic::tan)
      .def("asin", &symbolic::asin)
      .def("acos", &symbolic::acos)
      .def("atan", &symbolic::atan)
      .def("atan2", &symbolic::atan2)
      .def("sinh", &symbolic::sinh)
      .def("cosh", &symbolic::cosh)
      .def("tanh", &symbolic::tanh)
      .def("min", &symbolic::min)
      .def("max", &symbolic::max)
      .def("ceil", &symbolic::ceil)
      .def("floor", &symbolic::floor);

  m.def("if_then_else", &symbolic::if_then_else);

  m.def("Jacobian", [](const Eigen::Ref<const VectorX<Expression>>& f,
                       const Eigen::Ref<const VectorX<Variable>>& vars) {
    return Jacobian(f, vars);
  });

  py::class_<Formula>(m, "Formula")
      .def("GetFreeVariables", &Formula::GetFreeVariables)
      .def("EqualTo", &Formula::EqualTo)
      .def("Evaluate",
           [](const Formula& self, const Environment::map& env) {
             return self.Evaluate(Environment{env});
           })
      .def("Substitute",
           [](const Formula& self, const Variable& var, const Expression& e) {
             return self.Substitute(var, e);
           })
      .def("Substitute",
           [](const Formula& self, const Variable& var1, const Variable& var2) {
             return self.Substitute(var1, var2);
           })
      .def("Substitute", [](const Formula& self, const Variable& var,
                            const double c) { return self.Substitute(var, c); })
      .def("Substitute",
           [](const Formula& self, const Substitution& s) {
             return self.Substitute(s);
           })
      .def("to_string", &Formula::to_string)
      .def("__str__", &Formula::to_string)
      .def("__repr__",
           [](const Formula& self) {
             return fmt::format("<Formula \"{}\">", self.to_string());
           })
      .def("__eq__", [](const Formula& self,
                        const Formula& other) { return self.EqualTo(other); })
      .def("__ne__", [](const Formula& self,
                        const Formula& other) { return !self.EqualTo(other); })
      .def("__hash__",
           [](const Formula& self) { return std::hash<Formula>{}(self); })
      .def_static("True", &Formula::True)
      .def_static("False", &Formula::False)
      .def("__nonzero__", [](const Formula&) {
        throw std::runtime_error(
            "You should not call `__nonzero__` on `Formula`. If you are trying "
            "to make a map with `Variable`, `Expression`, or `Polynomial` as "
            "keys and access the keys, please use "
            "`pydrake.util.containers.EqualToDict`.");
      });

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
      .def(py::self *= py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__hash__",
           [](const Monomial& self) { return std::hash<Monomial>{}(self); })
      .def(py::self != py::self)
      .def("__str__",
           [](const Monomial& self) { return fmt::format("{}", self); })
      .def("__repr__",
           [](const Monomial& self) {
             return fmt::format("<Monomial \"{}\">", self);
           })
      .def("EqualTo", [](const Monomial& self,
                         const Monomial& monomial) { return self == monomial; })
      .def("GetVariables", &Monomial::GetVariables)
      .def("get_powers", &Monomial::get_powers, py_reference_internal)
      .def("ToExpression", &Monomial::ToExpression)
      .def("Evaluate",
           [](const Monomial& self, const Environment::map& env) {
             return self.Evaluate(Environment{env});
           })
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
      .def(py::self != py::self)
      .def("__hash__",
           [](const Polynomial& self) { return std::hash<Polynomial>{}(self); })
      .def("__str__",
           [](const Polynomial& self) { return fmt::format("{}", self); })
      .def("__repr__",
           [](const Polynomial& self) {
             return fmt::format("<Polynomial \"{}\">", self);
           })
      .def("__pow__",
           [](const Polynomial& self, const int n) { return pow(self, n); })
      .def("Evaluate",
           [](const Polynomial& self, const Environment::map& env) {
             return self.Evaluate(Environment{env});
           })
      .def("Jacobian", [](const Polynomial& p,
                          const Eigen::Ref<const VectorX<Variable>>& vars) {
        return p.Jacobian(vars);
      });

  // We have this line because pybind11 does not permit transitive
  // conversions. See
  // https://github.com/pybind/pybind11/blob/289e5d9cc2a4545d832d3c7fb50066476bce3c1d/include/pybind11/pybind11.h#L1629.
  py::implicitly_convertible<int, drake::symbolic::Expression>();
  py::implicitly_convertible<double, drake::symbolic::Expression>();
  py::implicitly_convertible<drake::symbolic::Variable,
                             drake::symbolic::Expression>();
  py::implicitly_convertible<drake::symbolic::Monomial,
                             drake::symbolic::Polynomial>();
}

}  // namespace pydrake
}  // namespace drake
