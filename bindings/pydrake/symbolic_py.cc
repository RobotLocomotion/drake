#include <map>
#include <string>

#include "fmt/format.h"
#include "fmt/ostream.h"
#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"

namespace drake {
namespace pydrake {

using std::map;
using std::string;

// TODO(eric.cousineau): Use py::self for operator overloads?
PYBIND11_MODULE(symbolic, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::symbolic;
  constexpr auto& doc = pydrake_doc.drake.symbolic;

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should aggressively try to avoid these warnings.
  py::module::import("pydrake.util.deprecation")
      .attr("install_numpy_warning_filters")();

  // Install NumPy formatters patch.
  py::module::import("pydrake.util.compatibility")
      .attr("maybe_patch_numpy_formatters")();

  m.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Variable> var_cls(m, "Variable", doc.Variable.doc);
  const auto& var_doc = doc.Variable;
  py::enum_<Variable::Type>(var_cls, "Type")
      .value(
          "CONTINUOUS", Variable::Type::CONTINUOUS, var_doc.Type.CONTINUOUS.doc)
      .value("INTEGER", Variable::Type::INTEGER, var_doc.Type.INTEGER.doc)
      .value("BINARY", Variable::Type::BINARY, var_doc.Type.BINARY.doc)
      .value("BOOLEAN", Variable::Type::BOOLEAN, var_doc.Type.BOOLEAN.doc)
      .value("RANDOM_UNIFORM", Variable::Type::RANDOM_UNIFORM,
          var_doc.Type.RANDOM_UNIFORM.doc)
      .value("RANDOM_GAUSSIAN", Variable::Type::RANDOM_GAUSSIAN,
          var_doc.Type.RANDOM_GAUSSIAN.doc)
      .value("RANDOM_EXPONENTIAL", Variable::Type::RANDOM_EXPONENTIAL,
          var_doc.Type.RANDOM_EXPONENTIAL.doc);

  var_cls
      .def(py::init<const string&, Variable::Type>(), py::arg("name"),
          py::arg("type") = Variable::Type::CONTINUOUS, var_doc.ctor.doc_2args)
      .def("get_id", &Variable::get_id, var_doc.get_id.doc)
      .def("get_type", &Variable::get_type, var_doc.get_type.doc)
      .def("__str__", &Variable::to_string, var_doc.to_string.doc)
      .def("__repr__",
          [](const Variable& self) {
            return fmt::format(
                "Variable('{}', {})", self.to_string(), self.get_type());
          })
      .def("__hash__",
          [](const Variable& self) { return std::hash<Variable>{}(self); })
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
      .def("EqualTo", &Variable::equal_to, var_doc.equal_to.doc)
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
  DefCopyAndDeepCopy(&var_cls);

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Variables>(m, "Variables", doc.Variables.doc)
      .def(py::init<>(), doc.Variables.ctor.doc_0args)
      .def(py::init<const Eigen::Ref<const VectorX<Variable>>&>(),
          doc.Variables.ctor.doc_1args_vec)
      .def("size", &Variables::size, doc.Variables.size.doc)
      .def("__len__", &Variables::size, doc.Variables.size.doc)
      .def("empty", &Variables::empty)
      .def("__str__", &Variables::to_string)
      .def("__repr__",
          [](const Variables& self) {
            return fmt::format("<Variables \"{}\">", self);
          })
      .def("to_string", &Variables::to_string, doc.Variables.to_string.doc)
      .def("__hash__",
          [](const Variables& self) { return std::hash<Variables>{}(self); })
      .def("insert",
          [](Variables& self, const Variable& var) { self.insert(var); },
          doc.Variables.insert.doc_1args_var)
      .def("insert",
          [](Variables& self, const Variables& vars) { self.insert(vars); },
          doc.Variables.insert.doc_1args_vars)
      .def("erase",
          [](Variables& self, const Variable& key) { return self.erase(key); },
          doc.Variables.erase.doc_1args_key)
      .def("erase",
          [](Variables& self, const Variables& vars) {
            return self.erase(vars);
          },
          doc.Variables.erase.doc_1args_vars)
      .def("include", &Variables::include, doc.Variables.include.doc)
      .def("__contains__", &Variables::include)
      .def("IsSubsetOf", &Variables::IsSubsetOf, doc.Variables.IsSubsetOf.doc)
      .def("IsSupersetOf", &Variables::IsSupersetOf,
          doc.Variables.IsSupersetOf.doc)
      .def("IsStrictSubsetOf", &Variables::IsStrictSubsetOf,
          doc.Variables.IsStrictSubsetOf.doc)
      .def("IsStrictSupersetOf", &Variables::IsStrictSupersetOf,
          doc.Variables.IsStrictSupersetOf.doc)
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

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  m.def("intersect", [](const Variables& vars1, const Variables& vars2) {
    return intersect(vars1, vars2);
  });

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Expression> expr_cls(m, "Expression");
  expr_cls.def(py::init<>(), doc.Expression.ctor.doc_0args)
      .def(py::init<double>(), doc.Expression.ctor.doc_1args_d)
      .def(py::init<const Variable&>(), doc.Expression.ctor.doc_1args_var)
      .def("__str__", &Expression::to_string)
      .def("__repr__",
          [](const Expression& self) {
            return fmt::format("<Expression \"{}\">", self.to_string());
          })
      .def(
          "__copy__", [](const Expression& self) -> Expression { return self; })
      .def("to_string", &Expression::to_string, doc.Expression.to_string.doc)
      .def("Expand", &Expression::Expand, doc.Expression.Expand.doc)
      .def("Evaluate",
          [](const Expression& self, const Environment::map& env,
              RandomGenerator* generator) {
            return self.Evaluate(Environment{env}, generator);
          },
          py::arg("env") = Environment::map{}, py::arg("generator") = nullptr,
          doc.Expression.Evaluate.doc_2args)
      .def("Evaluate",
          [](const Expression& self, RandomGenerator* generator) {
            return self.Evaluate(generator);
          },
          py::arg("generator"), doc.Expression.Evaluate.doc_1args)
      .def("EvaluatePartial",
          [](const Expression& self, const Environment::map& env) {
            return self.EvaluatePartial(Environment{env});
          },
          doc.Expression.EvaluatePartial.doc)
      .def("Substitute",
          [](const Expression& self, const Variable& var, const Expression& e) {
            return self.Substitute(var, e);
          },
          doc.Expression.Substitute.doc_2args)
      .def("Substitute",
          [](const Expression& self, const Substitution& s) {
            return self.Substitute(s);
          },
          doc.Expression.Substitute.doc_1args)
      .def("EqualTo", &Expression::EqualTo, doc.Expression.EqualTo.doc)
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
      .def("Differentiate", &Expression::Differentiate,
          doc.Expression.Differentiate.doc)
      .def("Jacobian", &Expression::Jacobian, doc.Expression.Jacobian.doc)
      // TODO(eric.cousineau): Figure out how to consolidate with the below
      // methods.
      .def("log", &symbolic::log, doc.log.doc)
      .def("__abs__", &symbolic::abs)
      .def("exp", &symbolic::exp, doc.exp.doc)
      .def("sqrt", &symbolic::sqrt, doc.sqrt.doc)
      // TODO(eric.cousineau): Move `__pow__` here.
      .def("sin", &symbolic::sin, doc.sin.doc)
      .def("cos", &symbolic::cos, doc.cos.doc)
      .def("tan", &symbolic::tan, doc.tan.doc)
      .def("arcsin", &symbolic::asin, doc.asin.doc)
      .def("arccos", &symbolic::acos, doc.acos.doc)
      .def("arctan2", &symbolic::atan2, doc.atan2.doc)
      .def("sinh", &symbolic::sinh, doc.sinh.doc)
      .def("cosh", &symbolic::cosh, doc.cosh.doc)
      .def("tanh", &symbolic::tanh, doc.tanh.doc)
      .def("min", &symbolic::min, doc.min.doc)
      .def("max", &symbolic::max, doc.max.doc)
      .def("ceil", &symbolic::ceil, doc.ceil.doc)
      .def("floor", &symbolic::floor, doc.floor.doc);
  DefCopyAndDeepCopy(&expr_cls);

  // TODO(eric.cousineau): Consider deprecating these methods?
  // TODO(m-chaturvedi) Add Pybind11 documentation.
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

  m.def("Jacobian",
      [](const Eigen::Ref<const VectorX<Expression>>& f,
          const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Jacobian(f, vars);
      },
      doc.Expression.Jacobian.doc);

  py::class_<Formula> formula_cls(m, "Formula", doc.Formula.doc);
  formula_cls
      .def("GetFreeVariables", &Formula::GetFreeVariables,
          doc.Formula.GetFreeVariables.doc)
      .def("EqualTo", &Formula::EqualTo, doc.Formula.EqualTo.doc)
      .def("Evaluate",
          [](const Formula& self, const Environment::map& env) {
            return self.Evaluate(Environment{env});
          },
          doc.Formula.Evaluate.doc_2args)
      .def("Substitute",
          [](const Formula& self, const Variable& var, const Expression& e) {
            return self.Substitute(var, e);
          },
          doc.Formula.Substitute.doc_2args)
      .def("Substitute",
          [](const Formula& self, const Variable& var1, const Variable& var2) {
            return self.Substitute(var1, var2);
          },
          doc.Formula.Substitute.doc_2args)
      .def("Substitute",
          [](const Formula& self, const Variable& var, const double c) {
            return self.Substitute(var, c);
          },
          doc.Formula.Substitute.doc_2args)
      .def("Substitute",
          [](const Formula& self, const Substitution& s) {
            return self.Substitute(s);
          },
          doc.Formula.Substitute.doc_1args)
      .def("to_string", &Formula::to_string, doc.Formula.to_string.doc)
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
      .def_static("True", &Formula::True, doc.FormulaTrue.doc)
      .def_static("False", &Formula::False, doc.FormulaFalse.doc)
      // `True` and `False` are reserved as of Python3
      .def_static("True_", &Formula::True, doc.FormulaTrue.doc)
      .def_static("False_", &Formula::False, doc.FormulaFalse.doc)
      .def("__nonzero__", [](const Formula&) {
        throw std::runtime_error(
            "You should not call `__bool__` / `__nonzero__` on `Formula`. "
            "If you are trying to make a map with `Variable`, `Expression`, "
            "or `Polynomial` as keys (and then access the map in Python), "
            "please use pydrake.util.containers.EqualToDict`.");
      });
  formula_cls.attr("__bool__") = formula_cls.attr("__nonzero__");

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

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Monomial>(m, "Monomial")
      .def(py::init<const Variable&>(), doc.Monomial.ctor.doc_1args_var)
      .def(py::init<const Variable&, int>(),
          doc.Monomial.ctor.doc_2args_var_exponent)
      .def(py::init<const map<Variable, int>&>(),
          doc.Monomial.ctor.doc_1args_powers)
      .def("degree", &Monomial::degree, doc.Monomial.degree.doc)
      .def("total_degree", &Monomial::total_degree,
          doc.Monomial.total_degree.doc)
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
      .def(
          "EqualTo", [](const Monomial& self,
                         const Monomial& monomial) { return self == monomial; })
      .def("GetVariables", &Monomial::GetVariables,
          doc.Monomial.GetVariables.doc)
      .def("get_powers", &Monomial::get_powers, py_reference_internal,
          doc.Monomial.get_powers.doc)
      .def("ToExpression", &Monomial::ToExpression,
          doc.Monomial.ToExpression.doc)
      .def("Evaluate",
          [](const Monomial& self, const Environment::map& env) {
            return self.Evaluate(Environment{env});
          },
          doc.Monomial.Evaluate.doc)
      .def("pow_in_place", &Monomial::pow_in_place, py_reference_internal,
          doc.Monomial.pow_in_place.doc)
      .def("__pow__",
          [](const Monomial& self, const int p) { return pow(self, p); });

  m  // BR
      .def("MonomialBasis",
          [](const Eigen::Ref<const VectorX<Variable>>& vars,
              const int degree) {
            return MonomialBasis(Variables{vars}, degree);
          },
          doc.MonomialBasis.doc_2args)
      .def("MonomialBasis",
          [](const Variables& vars, const int degree) {
            return MonomialBasis(vars, degree);
          },
          doc.MonomialBasis.doc_2args);

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Polynomial>(m, "Polynomial", doc.Polynomial.doc)
      .def(py::init<>(), doc.Polynomial.ctor.doc_0args)
      .def(py::init<Polynomial::MapType>(), doc.Polynomial.ctor.doc_1args_init)
      .def(py::init<const Monomial&>(), doc.Polynomial.ctor.doc_1args_m)
      .def(py::init<const Expression&>(), doc.Polynomial.ctor.doc_1args_e)
      .def(py::init<const Expression&, const Variables&>(),
          doc.Polynomial.ctor.doc_2args_e_indeterminates)
      .def(py::init([](const Expression& e,
                        const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Polynomial{e, Variables{vars}};
      }),
          doc.Polynomial.ctor.doc_2args_e_indeterminates)
      .def("indeterminates", &Polynomial::indeterminates,
          doc.Polynomial.indeterminates.doc, doc.Polynomial.indeterminates.doc)
      .def("decision_variables", &Polynomial::decision_variables,
          doc.Polynomial.decision_variables.doc)
      .def("Degree", &Polynomial::Degree, doc.Polynomial.Degree.doc)
      .def("TotalDegree", &Polynomial::TotalDegree,
          doc.Polynomial.TotalDegree.doc)
      .def("monomial_to_coefficient_map",
          &Polynomial::monomial_to_coefficient_map,
          doc.Polynomial.monomial_to_coefficient_map.doc)
      .def("ToExpression", &Polynomial::ToExpression,
          doc.Polynomial.ToExpression.doc)
      .def("Differentiate", &Polynomial::Differentiate,
          doc.Polynomial.Differentiate.doc)
      .def("AddProduct", &Polynomial::AddProduct, doc.Polynomial.AddProduct.doc)
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
      .def("EqualTo", &Polynomial::EqualTo, doc.Polynomial.EqualTo.doc)
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
          },
          doc.Polynomial.Evaluate.doc)
      .def("Jacobian",
          [](const Polynomial& p,
              const Eigen::Ref<const VectorX<Variable>>& vars) {
            return p.Jacobian(vars);
          },
          doc.Polynomial.Jacobian.doc);

  // We have this line because pybind11 does not permit transitive
  // conversions. See
  // https://github.com/pybind/pybind11/blob/289e5d9cc2a4545d832d3c7fb50066476bce3c1d/include/pybind11/pybind11.h#L1629.
  py::implicitly_convertible<int, drake::symbolic::Expression>();
  py::implicitly_convertible<double, drake::symbolic::Expression>();
  py::implicitly_convertible<drake::symbolic::Variable,
      drake::symbolic::Expression>();
  py::implicitly_convertible<drake::symbolic::Monomial,
      drake::symbolic::Polynomial>();

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
