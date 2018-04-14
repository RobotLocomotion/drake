#include <map>
#include <string>

#include "fmt/format.h"
#include "fmt/ostream.h"
#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/numpy_dtypes_pybind.h"
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

  // Predeclare all custom dtypes.
  py::dtype_user<Variable> var_cls(m, "Variable");
  py::dtype_user<Expression> expr_cls(m, "Expression");
  py::dtype_user<Formula> formula_cls(m, "Formula");

  var_cls
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
      .def_loop(py::self + py::self)
      .def_loop(py::self + double())
      .def_loop(double() + py::self)
      // Subtraction.
      .def_loop(py::self - py::self)
      .def_loop(py::self - double())
      .def_loop(double() - py::self)
      // Multiplication.
      .def_loop(py::self * py::self)
      .def_loop(py::self * double())
      .def_loop(double() * py::self)
      // Division.
      .def_loop(py::self / py::self)
      .def_loop(py::self / double())
      .def_loop(double() / py::self)
      // N.B. Since arithmetic is not closed for `Variable`, we cannot define
      // `dot` for NumPy since it expects a closed operation.
      // TODO(eric.cousineau): See if `dot` can be defined at some point.
      // Pow.
      .def_loop("__pow__",
           [](const Variable& self, double other) { return pow(self, other); })
      .def_loop("__pow__",
           [](const Variable& self, const Variable& other) {
             return pow(self, other);
           })
      .def_loop("__pow__",
           [](const Variable& self, const Expression& other) {
             return pow(self, other);
           })
      // See comment about `np.square` in AutoDiff<> bindings.
      .def_loop("square", [](const Variable& self) {
        return self * self;
      })
      // We add `EqualTo` instead of `equal_to` to maintain consistency among
      // symbolic classes (Variable, Expression, Formula, Polynomial) on Python
      // side. This enables us to achieve polymorphism via ducktyping in Python.
      .def("EqualTo", &Variable::equal_to)
      // Unary Plus.
      .def(+py::self)  // Not present in NumPy?
      // Unary Minus.
      .def_loop(-py::self)
      // LT(<).
      // Note that while pybind reflects `double < Variable`, NumPy UFunc loops
      // require both orders to be explicitly specified.
      .def_loop(py::self < Expression())
      .def_loop(py::self < py::self)
      .def_loop(py::self < double())
      .def_loop(double() < py::self, py::dtype_method::ufunc_only())
      // LE(<=).
      .def_loop(py::self <= Expression())
      .def_loop(py::self <= py::self)
      .def_loop(py::self <= double())
      .def_loop(double() <= py::self, py::dtype_method::ufunc_only())
      // GT(>).
      .def_loop(py::self > Expression())
      .def_loop(py::self > py::self)
      .def_loop(py::self > double())
      .def_loop(double() > py::self, py::dtype_method::ufunc_only())
      // GE(>=).
      .def_loop(py::self >= Expression())
      .def_loop(py::self >= py::self)
      .def_loop(py::self >= double())
      .def_loop(double() >= py::self, py::dtype_method::ufunc_only())
      // EQ(==).
      .def_loop(py::self == Expression())
      .def_loop(py::self == py::self)
      .def_loop(py::self == double())
      // NE(!=).
      .def_loop(py::self != Expression())
      .def_loop(py::self != py::self)
      .def_loop(py::self != double());

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

  DefImplicitConversionsFromNumericTypes(&expr_cls);
  expr_cls
      .def(py::init<>())
      .def(py::init<double>())
      .def(py::init<const Variable&>())
      // Casting
      .def_loop(py::dtype_method::implicit_conversion<Variable, Expression>())
      // Methods
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
      // Addition
      .def_loop(py::self + py::self)
      .def_loop(py::self + Variable())
      .def_loop(py::self + double())
      .def_loop(Variable() + py::self)
      .def_loop(double() + py::self)
      .def(py::self += py::self)
      .def(py::self += Variable())
      .def(py::self += double())
      // Subtraction.
      .def_loop(py::self - py::self)
      .def_loop(py::self - Variable())
      .def_loop(py::self - double())
      .def_loop(Variable() - py::self)
      .def_loop(double() - py::self)
      .def(py::self -= py::self)
      .def(py::self -= Variable())
      .def(py::self -= double())
      // Multiplication.
      .def_loop(py::self * py::self)
      .def_loop(py::self * Variable())
      .def_loop(py::self * double())
      .def_loop(Variable() * py::self)
      .def_loop(double() * py::self)
      .def(py::self *= py::self)
      .def(py::self *= Variable())
      .def(py::self *= double())
      // Division.
      .def_loop(py::self / py::self)
      .def_loop(py::self / Variable())
      .def_loop(py::self / double())
      .def_loop(Variable() / py::self)
      .def_loop(double() / py::self)
      .def(py::self /= py::self)
      .def(py::self /= Variable())
      .def(py::self /= double())
      // Dot-product.
      .def_loop(py::dtype_method::dot())
      // Unary Plus.
      .def(+py::self)  // Not present in NumPy?
      // Unary Minus.
      .def_loop(-py::self)
      // LT(<).
      // See notes for `Variable` about reversible operations.
      .def_loop(py::self < py::self)
      .def_loop(py::self < Variable())
      .def_loop(py::self < double())
      .def_loop(double() < py::self, py::dtype_method::ufunc_only())
      // LE(<=).
      .def_loop(py::self <= py::self)
      .def_loop(py::self <= Variable())
      .def_loop(py::self <= double())
      .def_loop(double() <= py::self, py::dtype_method::ufunc_only())
      // GT(>).
      .def_loop(py::self > py::self)
      .def_loop(py::self > Variable())
      .def_loop(py::self > double())
      .def_loop(double() > py::self, py::dtype_method::ufunc_only())
      // GE(>=).
      .def_loop(py::self >= py::self)
      .def_loop(py::self >= Variable())
      .def_loop(py::self >= double())
      .def_loop(double() >= py::self, py::dtype_method::ufunc_only())
      // EQ(==).
      .def_loop(py::self == py::self)
      .def_loop(py::self == Variable())
      .def_loop(py::self == double())
      // NE(!=)
      .def_loop(py::self != py::self)
      .def_loop(py::self != Variable())
      .def_loop(py::self != double())
      // See comment about `np.square` in AutoDiff<> bindings.
      .def_loop("square", [](const Expression& self) {
        return self * self;
      })
      .def("Differentiate", &Expression::Differentiate)
      .def("Jacobian", &Expression::Jacobian);

  // Define a `self` method and a ufunc flavor of certain methods.
  // TODO(eric.cousineau): Figure out how to make this play well with `self`
  // overloads. Bind the ufunc as a method?
  // TODO(eric.cousineau): Consider defining a general `equal_to` method, with
  // overloads for other types.
  expr_cls.def("EqualTo", &Expression::EqualTo);
  py::ufunc(expr_cls, "equal_to").def_loop<Expression>(&Expression::EqualTo);

  // TODO(eric.cousineau): Consider deprecating the aliases in `math`?
  auto math = py::module::import("pydrake.math");
  UfuncMirrorDef<decltype(expr_cls)>(&expr_cls, math)
      // TODO(eric.cousineau): Figure out how to consolidate with the below
      // methods.
      // Pow.
      .def_loop("__pow__", "pow", [](const Expression& self,
                         const double other) { return pow(self, other); })
      .def_loop("__pow__", "pow", [](const Expression& self,
                         const Variable& other) { return pow(self, other); })
      .def_loop("__pow__", "pow", [](const Expression& self,
                         const Expression& other) { return pow(self, other); })
      .def_loop("log", &symbolic::log)
      .def_loop("__abs__", "abs", &symbolic::abs)
      .def_loop("exp", &symbolic::exp)
      .def_loop("sqrt", &symbolic::sqrt)
      // TODO(eric.cousineau): Move `__pow__` here.
      .def_loop("sin", &symbolic::sin)
      .def_loop("cos", &symbolic::cos)
      .def_loop("tan", &symbolic::tan)
      .def_loop("arcsin", "asin", &symbolic::asin)
      .def_loop("arccos", "acos", &symbolic::acos)
      .def_loop("arctan2", "atan2", &symbolic::atan2)
      .def_loop("sinh", &symbolic::sinh)
      .def_loop("cosh", &symbolic::cosh)
      .def_loop("tanh", &symbolic::tanh)
      .def_loop("fmin", "min", &symbolic::min)
      .def_loop("fmax", "max", &symbolic::max)
      .def_loop("ceil", &symbolic::ceil)
      .def_loop("floor", &symbolic::floor);

  MirrorDef<decltype(expr_cls), py::module>(&expr_cls, &math)
      .def("atan", &symbolic::atan);

  // Import aliases.
  // TODO(eric.cousineau): Deprecate, then remove these in lieu of `np.{func}`
  py::exec(R"""(
from pydrake.math import (
    log,
    abs,
    exp,
    pow,
    sqrt,
    sin,
    cos,
    tan,
    asin,
    acos,
    atan,
    atan2,
    sinh,
    cosh,
    tanh,
    min,
    max,
    ceil,
    floor
)
)""");

  m.def("if_then_else", [](bool cond, double true_value, double false_value) {
    return cond ? true_value : false_value;
  });
  m.def("if_then_else", &symbolic::if_then_else);

  m.def("Jacobian", [](const Eigen::Ref<const VectorX<Expression>>& f,
                       const Eigen::Ref<const VectorX<Variable>>& vars) {
    return Jacobian(f, vars);
  });

  formula_cls
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
      .def_loop("__eq__", [](const Formula& self,
                        const Formula& other) { return self.EqualTo(other); })
      .def("__ne__", [](const Formula& self,
                        const Formula& other) { return !self.EqualTo(other); })
      .def("__hash__",
           [](const Formula& self) { return std::hash<Formula>{}(self); })
      .def("__nonzero__", [](const Formula&) {
        throw std::runtime_error(
            "You should not call `__nonzero__` on `Formula`. If you are trying "
            "to make a map with `Variable`, `Expression`, or `Polynomial` as "
            "keys and access the keys, please use "
            "`pydrake.util.containers.EqualToDict`.");
      });
  formula_cls.cls()
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

  py::implicitly_convertible<drake::symbolic::Monomial,
                             drake::symbolic::Polynomial>();
}

}  // namespace pydrake
}  // namespace drake
