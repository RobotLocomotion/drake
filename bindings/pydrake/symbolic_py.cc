#include <map>
#include <string>

#include "fmt/format.h"
#include "fmt/ostream.h"
#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/numpy_dtypes_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

#pragma GCC diagnostic push
// Apple LLVM version 10.0.1 (clang-1001.0.46.3) adds `-Wself-assign-overloaded`
// to `-Wall`, which generates warnings on Pybind11's operator-overloading idiom
// that is using py::self (example: `def(py::self + py::self)`).
// Here, we suppress the warning using `#pragma diagnostic`.
#if (__APPLE__) && (__clang__) && (__clang_major__ >= 10) && \
    (__clang_minor__ >= 0) && (__clang_patchlevel__ >= 1)
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

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
  py::module::import("pydrake.common.deprecation")
      .attr("install_numpy_warning_filters")();

  // Install NumPy formatters patch.
  py::module::import("pydrake.common.compatibility")
      .attr("maybe_patch_numpy_formatters")();

  m.doc() =
      "Symbolic variable, variables, monomial, expression, polynomial, and "
      "formula";

  const auto& var_doc = doc.Variable;
  // Predeclare all custom dtypes.
  py::dtype_user<Variable> var_cls(m, "Variable", doc.Variable.doc);
  py::dtype_user<Expression> expr_cls(m, "Expression", doc.Expression.doc);
  py::dtype_user<Formula> formula_cls(m, "Formula", doc.Formula.doc);

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
      .def(py::init<const string&, Variable::Type>(),
          py::arg("name"), py::arg("type") = Variable::Type::CONTINUOUS,
          var_doc.ctor.doc_2args)
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
      .def_loop("__pow__", [](const Variable& self,
                              double other) { return pow(self, other); })
      .def_loop("__pow__",
                [](const Variable& self, const Variable& other) {
                  return pow(self, other);
                })
      .def_loop("__pow__",
                [](const Variable& self, const Expression& other) {
                  return pow(self, other);
                })
      // See comment about `np.square` in AutoDiff<> bindings.
      .def_loop("square", [](const Variable& self) { return self * self; })
      // We add `EqualTo` instead of `equal_to` to maintain consistency among
      // symbolic classes (Variable, Expression, Formula, Polynomial) on Python
      // side. This enables us to achieve polymorphism via ducktyping in Python.
      .def("EqualTo", &Variable::equal_to, var_doc.equal_to.doc)
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
  DefCopyAndDeepCopy(&var_cls.cls());

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

  DefImplicitConversionsFromNumericTypes(&expr_cls);
  expr_cls  // BR
      .def(py::init<>(), doc.Expression.ctor.doc_0args)
      .def(py::init<double>(), doc.Expression.ctor.doc_1args_d)
      .def(py::init<const Variable&>(), doc.Expression.ctor.doc_1args_var)
      // Casting
      .def_loop(py::dtype_method::implicit_conversion<Variable, Expression>())
      // Methods
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
      .def("GetVariables", &Expression::GetVariables,
          doc.Expression.GetVariables.doc)
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
      .def_loop("square", [](const Expression& self) { return self * self; })
      .def("Differentiate", &Expression::Differentiate,
           doc.Expression.Differentiate.doc)
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
      .def_loop("__pow__", "pow",
                [](const Expression& self, const double other) {
                  return pow(self, other);
                })
      .def_loop("__pow__", "pow",
                [](const Expression& self, const Variable& other) {
                  return pow(self, other);
                })
      .def_loop("__pow__", "pow",
                [](const Expression& self, const Expression& other) {
                  return pow(self, other);
                })
      .def_loop("log", &symbolic::log)
      .def_loop("__abs__", "abs", &symbolic::abs)
      .def_loop("exp", &symbolic::exp)
      .def_loop("sqrt", &symbolic::sqrt)
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
      .def("atan", &symbolic::atan)
      // Matrix overloads.
      .def("inv", [](const MatrixX<Expression>& X) -> MatrixX<Expression> {
        return X.inverse();
      });

  m.def("if_then_else", [](bool cond, double true_value, double false_value) {
    return cond ? true_value : false_value;
  });
  m.def("if_then_else", &symbolic::if_then_else);

  m.def("Jacobian",
      [](const Eigen::Ref<const VectorX<Expression>>& f,
          const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Jacobian(f, vars);
      },
      doc.Expression.Jacobian.doc);

  m.def("Evaluate",
      [](const MatrixX<Expression>& M, const Environment::map& env,
          RandomGenerator* random_generator) {
        return Evaluate(M, Environment{env}, random_generator);
      },
      py::arg("m"), py::arg("env") = Environment::map{},
      py::arg("generator") = nullptr, doc.Evaluate.doc);

  m.def("Substitute",
      [](const MatrixX<Expression>& M, const Substitution& subst) {
        return Substitute(M, subst);
      },
      py::arg("m"), py::arg("subst"), doc.Substitute.doc_2args);

  m.def("Substitute",
      [](const MatrixX<Expression>& M, const Variable& var,
          const Expression& e) { return Substitute(M, var, e); },
      py::arg("m"), py::arg("var"), py::arg("e"), doc.Substitute.doc_3args);

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
      .def_loop("__eq__",
                [](const Formula& self, const Formula& other) {
                  return self.EqualTo(other);
                })
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
            "please use pydrake.common.containers.EqualToDict`.");
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
      .def("RemoveTermsWithSmallCoefficients",
          &Polynomial::RemoveTermsWithSmallCoefficients,
          py::arg("coefficient_tol"),
          doc.Polynomial.RemoveTermsWithSmallCoefficients.doc)
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

  py::implicitly_convertible<drake::symbolic::Monomial,
      drake::symbolic::Polynomial>();

  ExecuteExtraPythonCode(m);
  // NOLINTNEXTLINE(readability/fn_size)
}

}  // namespace pydrake
}  // namespace drake

#pragma GCC diagnostic pop
