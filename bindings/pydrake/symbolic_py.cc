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
#include "drake/bindings/pydrake/symbolic_py_unapply.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/common/symbolic_decompose.h"

#pragma GCC diagnostic push
// Apple LLVM version 10.0.1 (clang-1001.0.46.3) and Clang version 7.0.0 add
// `-Wself-assign-overloaded` to `-Wall`, which generates warnings on
// Pybind11's operator-overloading idiom that is using py::self (example:
// `def(py::self + py::self)`). Here, we suppress the warning using
// `#pragma diagnostic`.
#if defined(__clang__) &&                                          \
    ((!(__apple_build_version__) && (__clang_major__ >= 7)) ||     \
        ((__apple_build_version__) && (__clang_major__ >= 10) &&   \
            !((__clang_major__ == 10) && (__clang_minor__ == 0) && \
                (__clang_patchlevel__ == 0))))
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

  py::module::import("pydrake.common");

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

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Variable> var_cls(m, "Variable", doc.Variable.doc);
  constexpr auto& var_doc = doc.Variable;
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
      .def("is_dummy", &Variable::is_dummy, var_doc.is_dummy.doc)
      .def("get_id", &Variable::get_id, var_doc.get_id.doc)
      .def("get_type", &Variable::get_type, var_doc.get_type.doc)
      .def("get_name", &Variable::get_name, var_doc.get_name.doc)
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
      .def(
          "__pow__",
          [](const Variable& self, double other) { return pow(self, other); },
          py::is_operator())
      .def(
          "__pow__",
          [](const Variable& self, const Variable& other) {
            return pow(self, other);
          },
          py::is_operator())
      .def(
          "__pow__",
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
  internal::BindSymbolicMathOverloads<Variable>(&var_cls);
  DefCopyAndDeepCopy(&var_cls);

  // Bind the free function TaylorExpand.
  m.def(
      "TaylorExpand",
      [](const symbolic::Expression& f, const symbolic::Environment::map& a,
          int order) {
        return symbolic::TaylorExpand(f, symbolic::Environment(a), order);
      },
      py::arg("f"), py::arg("a"), py::arg("order"), doc.TaylorExpand.doc);

  // Bind the free functions for Make(Vector|Matrix)(...)Variable.
  m  // BR
      .def(
          "MakeMatrixVariable",
          [](int rows, int cols, const std::string& name, Variable::Type type) {
            return symbolic::MakeMatrixVariable(rows, cols, name, type);
          },
          py::arg("rows"), py::arg("cols"), py::arg("name"),
          py::arg("type") = symbolic::Variable::Type::CONTINUOUS,
          doc.MakeMatrixVariable.doc_4args)
      .def(
          "MakeMatrixBinaryVariable",
          [](int rows, int cols, const std::string& name) {
            return symbolic::MakeMatrixBinaryVariable(rows, cols, name);
          },
          py::arg("rows"), py::arg("cols"), py::arg("name"),
          doc.MakeMatrixBinaryVariable.doc_3args)
      .def(
          "MakeMatrixContinuousVariable",
          [](int rows, int cols, const std::string& name) {
            return symbolic::MakeMatrixContinuousVariable(rows, cols, name);
          },
          py::arg("rows"), py::arg("cols"), py::arg("name"),
          doc.MakeMatrixContinuousVariable.doc_3args)
      .def(
          "MakeMatrixBooleanVariable",
          [](int rows, int cols, const std::string& name) {
            return symbolic::MakeMatrixBooleanVariable(rows, cols, name);
          },
          py::arg("rows"), py::arg("cols"), py::arg("name"),
          doc.MakeMatrixBooleanVariable.doc_3args)
      .def(
          "MakeVectorVariable",
          [](int rows, const std::string& name, Variable::Type type) {
            return symbolic::MakeVectorVariable(rows, name, type);
          },
          py::arg("rows"), py::arg("name"),
          py::arg("type") = symbolic::Variable::Type::CONTINUOUS,
          doc.MakeVectorVariable.doc_3args)
      .def(
          "MakeVectorBinaryVariable",
          [](int rows, const std::string& name) {
            return symbolic::MakeVectorBinaryVariable(rows, name);
          },
          py::arg("rows"), py::arg("name"),
          doc.MakeVectorBinaryVariable.doc_2args)
      .def(
          "MakeVectorBooleanVariable",
          [](int rows, const std::string& name) {
            return symbolic::MakeVectorBooleanVariable(rows, name);
          },
          py::arg("rows"), py::arg("name"),
          doc.MakeVectorBooleanVariable.doc_2args)
      .def(
          "MakeVectorContinuousVariable",
          [](int rows, const std::string& name) {
            return symbolic::MakeVectorContinuousVariable(rows, name);
          },
          py::arg("rows"), py::arg("name"),
          doc.MakeVectorContinuousVariable.doc_2args);

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads,
  // etc.
  py::class_<Variables>(m, "Variables", doc.Variables.doc)
      .def(py::init<>(), doc.Variables.ctor.doc_0args)
      .def(py::init<const Eigen::Ref<const VectorX<Variable>>&>(),
          doc.Variables.ctor.doc_1args_vec)
      .def("size", &Variables::size, doc.Variables.size.doc)
      .def("__len__", &Variables::size, doc.Variables.size.doc)
      .def("empty", &Variables::empty, doc.Variables.empty.doc)
      .def("__str__", &Variables::to_string, doc.Variables.to_string.doc)
      .def("__repr__",
          [](const Variables& self) {
            return fmt::format("<Variables \"{}\">", self);
          })
      .def("to_string", &Variables::to_string, doc.Variables.to_string.doc)
      .def("__hash__",
          [](const Variables& self) { return std::hash<Variables>{}(self); })
      .def(
          "insert",
          [](Variables& self, const Variable& var) { self.insert(var); },
          py::arg("var"), doc.Variables.insert.doc_1args_var)
      .def(
          "insert",
          [](Variables& self, const Variables& vars) { self.insert(vars); },
          py::arg("vars"), doc.Variables.insert.doc_1args_vars)
      .def(
          "erase",
          [](Variables& self, const Variable& key) { return self.erase(key); },
          py::arg("key"), doc.Variables.erase.doc_1args_key)
      .def(
          "erase",
          [](Variables& self, const Variables& vars) {
            return self.erase(vars);
          },
          py::arg("vars"), doc.Variables.erase.doc_1args_vars)
      .def("include", &Variables::include, py::arg("key"),
          doc.Variables.include.doc)
      .def("__contains__", &Variables::include)
      .def("IsSubsetOf", &Variables::IsSubsetOf, py::arg("vars"),
          doc.Variables.IsSubsetOf.doc)
      .def("IsSupersetOf", &Variables::IsSupersetOf, py::arg("vars"),
          doc.Variables.IsSupersetOf.doc)
      .def("IsStrictSubsetOf", &Variables::IsStrictSubsetOf, py::arg("vars"),
          doc.Variables.IsStrictSubsetOf.doc)
      .def("IsStrictSupersetOf", &Variables::IsStrictSupersetOf,
          py::arg("vars"), doc.Variables.IsStrictSupersetOf.doc)
      .def("EqualTo", [](const Variables& self,
                          const Variables& vars) { return self == vars; })
      .def(
          "__iter__",
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

  m.def(
      "intersect",
      [](const Variables& vars1, const Variables& vars2) {
        return intersect(vars1, vars2);
      },
      py::arg("vars1"), py::arg("vars2"), doc.intersect.doc);

  {
    constexpr auto& cls_doc = doc.ExpressionKind;
    py::enum_<ExpressionKind>(m, "ExpressionKind", doc.ExpressionKind.doc)
        .value("Constant", ExpressionKind::Constant, cls_doc.Constant.doc)
        .value("Var", ExpressionKind::Var, cls_doc.Var.doc)
        .value("Add", ExpressionKind::Add, cls_doc.Add.doc)
        .value("Mul", ExpressionKind::Mul, cls_doc.Mul.doc)
        .value("Div", ExpressionKind::Div, cls_doc.Div.doc)
        .value("Log", ExpressionKind::Log, cls_doc.Log.doc)
        .value("Abs", ExpressionKind::Abs, cls_doc.Abs.doc)
        .value("Exp", ExpressionKind::Exp, cls_doc.Exp.doc)
        .value("Sqrt", ExpressionKind::Sqrt, cls_doc.Sqrt.doc)
        .value("Pow", ExpressionKind::Pow, cls_doc.Pow.doc)
        .value("Sin", ExpressionKind::Sin, cls_doc.Sin.doc)
        .value("Cos", ExpressionKind::Cos, cls_doc.Cos.doc)
        .value("Tan", ExpressionKind::Tan, cls_doc.Tan.doc)
        .value("Asin", ExpressionKind::Asin, cls_doc.Asin.doc)
        .value("Acos", ExpressionKind::Acos, cls_doc.Acos.doc)
        .value("Atan", ExpressionKind::Atan, cls_doc.Atan.doc)
        .value("Atan2", ExpressionKind::Atan2, cls_doc.Atan2.doc)
        .value("Sinh", ExpressionKind::Sinh, cls_doc.Sinh.doc)
        .value("Cosh", ExpressionKind::Cosh, cls_doc.Cosh.doc)
        .value("Tanh", ExpressionKind::Tanh, cls_doc.Tanh.doc)
        .value("Min", ExpressionKind::Min, cls_doc.Min.doc)
        .value("Max", ExpressionKind::Max, cls_doc.Max.doc)
        .value("Ceil", ExpressionKind::Ceil, cls_doc.Ceil.doc)
        .value("Floor", ExpressionKind::Floor, cls_doc.Floor.doc)
        .value("IfThenElse", ExpressionKind::IfThenElse, cls_doc.IfThenElse.doc)
        .value("NaN", ExpressionKind::NaN, cls_doc.NaN.doc)
        .value("UninterpretedFunction", ExpressionKind::UninterpretedFunction,
            cls_doc.UninterpretedFunction.doc);
  }

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Expression> expr_cls(m, "Expression", doc.Expression.doc);
  expr_cls.def(py::init<>(), doc.Expression.ctor.doc_0args)
      .def(py::init<double>(), py::arg("constant"),
          doc.Expression.ctor.doc_1args_constant)
      .def(py::init<const Variable&>(), py::arg("var"),
          doc.Expression.ctor.doc_1args_var)
      .def("__str__", &Expression::to_string, doc.Expression.to_string.doc)
      .def("__repr__",
          [](const Expression& self) {
            return fmt::format("<Expression \"{}\">", self.to_string());
          })
      .def(
          "__copy__", [](const Expression& self) -> Expression { return self; })
      .def("get_kind", &Expression::get_kind, doc.Expression.get_kind.doc)
      .def("to_string", &Expression::to_string, doc.Expression.to_string.doc)
      .def(
          "Unapply",
          [m](const symbolic::Expression& e) {
            return internal::Unapply(m, e);
          },
          internal::kUnapplyExpressionDoc)
      .def("Expand", &Expression::Expand, doc.Expression.Expand.doc)
      .def(
          "Evaluate",
          [](const Expression& self, const Environment::map& env,
              RandomGenerator* generator) {
            return self.Evaluate(Environment{env}, generator);
          },
          py::arg("env") = Environment::map{}, py::arg("generator") = nullptr,
          doc.Expression.Evaluate.doc_2args)
      .def(
          "Evaluate",
          [](const Expression& self, RandomGenerator* generator) {
            return self.Evaluate(generator);
          },
          py::arg("generator"), doc.Expression.Evaluate.doc_1args)
      .def(
          "EvaluatePartial",
          [](const Expression& self, const Environment::map& env) {
            return self.EvaluatePartial(Environment{env});
          },
          py::arg("env"), doc.Expression.EvaluatePartial.doc)
      .def("GetVariables", &Expression::GetVariables,
          doc.Expression.GetVariables.doc)
      .def(
          "Substitute",
          [](const Expression& self, const Variable& var, const Expression& e) {
            return self.Substitute(var, e);
          },
          py::arg("var"), py::arg("e"), doc.Expression.Substitute.doc_2args)
      .def(
          "Substitute",
          [](const Expression& self, const Substitution& s) {
            return self.Substitute(s);
          },
          py::arg("s"), doc.Expression.Substitute.doc_1args)
      .def("EqualTo", &Expression::EqualTo, doc.Expression.EqualTo.doc)
      .def("is_polynomial", &Expression::is_polynomial,
          doc.Expression.is_polynomial.doc)
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
      .def("Differentiate", &Expression::Differentiate, py::arg("x"),
          doc.Expression.Differentiate.doc)
      .def("Jacobian", &Expression::Jacobian, py::arg("vars"),
          doc.Expression.Jacobian.doc);
  // TODO(eric.cousineau): Clean this overload stuff up (#15041).
  pydrake::internal::BindSymbolicMathOverloads<Expression>(&expr_cls);
  pydrake::internal::BindSymbolicMathOverloads<Expression>(&m);
  DefCopyAndDeepCopy(&expr_cls);

  m.def("if_then_else", &symbolic::if_then_else, py::arg("f_cond"),
      py::arg("e_then"), py::arg("e_else"), doc.if_then_else.doc);
  m.def("uninterpreted_function", &symbolic::uninterpreted_function,
      py::arg("name"), py::arg("arguments"), doc.uninterpreted_function.doc);

  m.def(
      "Jacobian",
      [](const Eigen::Ref<const VectorX<Expression>>& f,
          const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Jacobian(f, vars);
      },
      py::arg("f"), py::arg("vars"), doc.Jacobian.doc);

  m.def(
      "IsAffine",
      [](const Eigen::Ref<const MatrixX<Expression>>& M,
          const Variables& vars) { return IsAffine(M, vars); },
      py::arg("m"), py::arg("vars"), doc.IsAffine.doc_2args);

  m.def(
      "IsAffine",
      [](const Eigen::Ref<const MatrixX<Expression>>& M) {
        return IsAffine(M);
      },
      py::arg("m"), doc.IsAffine.doc_1args);

  m.def(
      "Evaluate",
      [](const MatrixX<Expression>& M, const Environment::map& env,
          RandomGenerator* random_generator) {
        return Evaluate(M, Environment{env}, random_generator);
      },
      py::arg("m"), py::arg("env") = Environment::map{},
      py::arg("generator") = nullptr, doc.Evaluate.doc_expression);

  m.def("GetVariableVector", &symbolic::GetVariableVector,
      py::arg("expressions"), doc.GetVariableVector.doc);

  m.def(
      "Substitute",
      [](const MatrixX<Expression>& M, const Substitution& subst) {
        return Substitute(M, subst);
      },
      py::arg("m"), py::arg("subst"), doc.Substitute.doc_2args);

  m.def(
      "Substitute",
      [](const MatrixX<Expression>& M, const Variable& var,
          const Expression& e) { return Substitute(M, var, e); },
      py::arg("m"), py::arg("var"), py::arg("e"), doc.Substitute.doc_3args);

  {
    constexpr auto& cls_doc = doc.FormulaKind;
    py::enum_<FormulaKind>(m, "FormulaKind", doc.FormulaKind.doc)
        .value("False", FormulaKind::False, cls_doc.False.doc)
        .value("True", FormulaKind::True, cls_doc.True.doc)
        .value("Var", FormulaKind::Var, cls_doc.Var.doc)
        .value("Eq", FormulaKind::Eq, cls_doc.Eq.doc)
        .value("Neq", FormulaKind::Neq, cls_doc.Neq.doc)
        .value("Gt", FormulaKind::Gt, cls_doc.Gt.doc)
        .value("Geq", FormulaKind::Geq, cls_doc.Geq.doc)
        .value("Lt", FormulaKind::Lt, cls_doc.Lt.doc)
        .value("Leq", FormulaKind::Leq, cls_doc.Leq.doc)
        .value("And", FormulaKind::And, cls_doc.And.doc)
        .value("Or", FormulaKind::Or, cls_doc.Or.doc)
        .value("Not", FormulaKind::Not, cls_doc.Not.doc)
        .value("Forall", FormulaKind::Forall, cls_doc.Forall.doc)
        .value("Isnan", FormulaKind::Isnan, cls_doc.Isnan.doc)
        .value("PositiveSemidefinite", FormulaKind::PositiveSemidefinite,
            cls_doc.PositiveSemidefinite.doc);
  }

  py::class_<Formula> formula_cls(m, "Formula", doc.Formula.doc);
  formula_cls.def(py::init<>(), doc.Expression.ctor.doc_0args)
      .def(py::init<const Variable&>(), py::arg("var"),
          doc.Expression.ctor.doc_1args_var)
      .def(
          "Unapply",
          [m](const symbolic::Formula& f) { return internal::Unapply(m, f); },
          internal::kUnapplyFormulaDoc)
      .def("get_kind", &Formula::get_kind, doc.Formula.get_kind.doc)
      .def("GetFreeVariables", &Formula::GetFreeVariables,
          doc.Formula.GetFreeVariables.doc)
      .def("EqualTo", &Formula::EqualTo, doc.Formula.EqualTo.doc)
      .def(
          "Evaluate",
          [](const Formula& self, const Environment::map& env) {
            return self.Evaluate(Environment{env});
          },
          py::arg("env") = Environment::map{}, doc.Formula.Evaluate.doc_2args)
      .def(
          "Substitute",
          [](const Formula& self, const Variable& var, const Expression& e) {
            return self.Substitute(var, e);
          },
          py::arg("var"), py::arg("e"), doc.Formula.Substitute.doc_2args)
      .def(
          "Substitute",
          [](const Formula& self, const Variable& var1, const Variable& var2) {
            return self.Substitute(var1, var2);
          },
          py::arg("var"), py::arg("e"), doc.Formula.Substitute.doc_2args)
      .def(
          "Substitute",
          [](const Formula& self, const Variable& var, const double c) {
            return self.Substitute(var, c);
          },
          py::arg("var"), py::arg("e"), doc.Formula.Substitute.doc_2args)
      .def(
          "Substitute",
          [](const Formula& self, const Substitution& s) {
            return self.Substitute(s);
          },
          py::arg("s"), doc.Formula.Substitute.doc_1args)
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

  m.def("isnan", &symbolic::isnan, py::arg("e"), doc.isnan.doc);
  m.def("forall", &symbolic::forall, py::arg("vars"), py::arg("f"),
      doc.forall.doc);
  m.def("positive_semidefinite",
      overload_cast_explicit<Formula,
          const Eigen::Ref<const MatrixX<Expression>>&>(
          &symbolic::positive_semidefinite),
      py::arg("m"), doc.positive_semidefinite.doc_1args_m);

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Monomial>(m, "Monomial", doc.Monomial.doc)
      .def(py::init<>(), doc.Monomial.ctor.doc_0args)
      .def(py::init<const Variable&>(), py::arg("var"),
          doc.Monomial.ctor.doc_1args_var)
      .def(py::init<const Variable&, int>(), py::arg("var"),
          py::arg("exponent"), doc.Monomial.ctor.doc_2args_var_exponent)
      .def(py::init<const map<Variable, int>&>(), py::arg("powers"),
          doc.Monomial.ctor.doc_1args_powers)
      .def(py::init<const Eigen::Ref<const VectorX<Variable>>&,
               const Eigen::Ref<const Eigen::VectorXi>&>(),
          py::arg("vars"), py::arg("exponents"),
          doc.Monomial.ctor.doc_2args_vars_exponents)
      .def("degree", &Monomial::degree, py::arg("v"), doc.Monomial.degree.doc)
      .def("total_degree", &Monomial::total_degree,
          doc.Monomial.total_degree.doc)
      .def(py::self * py::self)
      .def(py::self *= py::self)
      .def(py::self * double{})
      .def(double{} * py::self)
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
      .def("get_powers", &Monomial::get_powers, py_rvp::reference_internal,
          doc.Monomial.get_powers.doc)
      .def("ToExpression", &Monomial::ToExpression,
          doc.Monomial.ToExpression.doc)
      .def(
          "Evaluate",
          [](const Monomial& self, const Environment::map& env) {
            return self.Evaluate(Environment{env});
          },
          py::arg("env"), doc.Monomial.Evaluate.doc)
      .def(
          "EvaluatePartial",
          [](const Monomial& self, const Environment::map& env) {
            return self.EvaluatePartial(Environment{env});
          },
          py::arg("env"), doc.Monomial.EvaluatePartial.doc)
      .def("pow_in_place", &Monomial::pow_in_place, py_rvp::reference_internal,
          py::arg("p"), doc.Monomial.pow_in_place.doc)
      .def("__pow__",
          [](const Monomial& self, const int p) { return pow(self, p); });

  m  // BR
      .def(
          "MonomialBasis",
          [](const Eigen::Ref<const VectorX<Variable>>& vars,
              const int degree) {
            return MonomialBasis(Variables{vars}, degree);
          },
          py::arg("vars"), py::arg("degree"), doc.MonomialBasis.doc_2args)
      .def(
          "MonomialBasis",
          [](const Variables& vars, const int degree) {
            return MonomialBasis(vars, degree);
          },
          py::arg("vars"), py::arg("degree"), doc.MonomialBasis.doc_2args)
      .def("EvenDegreeMonomialBasis", &symbolic::EvenDegreeMonomialBasis,
          py::arg("vars"), py::arg("degree"), doc.EvenDegreeMonomialBasis.doc)
      .def("OddDegreeMonomialBasis", &symbolic::OddDegreeMonomialBasis,
          py::arg("vars"), py::arg("degree"), doc.OddDegreeMonomialBasis.doc);

  using symbolic::Polynomial;

  // TODO(m-chaturvedi) Add Pybind11 documentation for operator overloads, etc.
  py::class_<Polynomial>(m, "Polynomial", doc.Polynomial.doc)
      .def(py::init<>(), doc.Polynomial.ctor.doc_0args)
      .def(py::init<Polynomial::MapType>(), py::arg("map"),
          doc.Polynomial.ctor.doc_1args_map)
      .def(py::init<const Monomial&>(), py::arg("m"),
          doc.Polynomial.ctor.doc_1args_m)
      .def(py::init<const Expression&>(), py::arg("e"),
          doc.Polynomial.ctor.doc_1args_e)
      .def(py::init<const Expression&, const Variables&>(), py::arg("e"),
          py::arg("indeterminates"),
          doc.Polynomial.ctor.doc_2args_e_indeterminates)
      .def(py::init([](const Expression& e,
                        const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Polynomial{e, Variables{vars}};
      }),
          py::arg("e"), py::arg("indeterminates"),
          doc.Polynomial.ctor.doc_2args_e_indeterminates)
      .def("indeterminates", &Polynomial::indeterminates,
          doc.Polynomial.indeterminates.doc)
      .def("decision_variables", &Polynomial::decision_variables,
          doc.Polynomial.decision_variables.doc)
      .def("SetIndeterminates", &Polynomial::SetIndeterminates,
          py::arg("new_indeterminates"), doc.Polynomial.SetIndeterminates.doc)
      .def("Degree", &Polynomial::Degree, py::arg("v"),
          doc.Polynomial.Degree.doc)
      .def("TotalDegree", &Polynomial::TotalDegree,
          doc.Polynomial.TotalDegree.doc)
      .def("monomial_to_coefficient_map",
          &Polynomial::monomial_to_coefficient_map,
          doc.Polynomial.monomial_to_coefficient_map.doc)
      .def("ToExpression", &Polynomial::ToExpression,
          doc.Polynomial.ToExpression.doc)
      .def("Differentiate", &Polynomial::Differentiate, py::arg("x"),
          doc.Polynomial.Differentiate.doc)
      .def(
          "Integrate",
          [](const Polynomial& self, const Variable& var) {
            return self.Integrate(var);
          },
          py::arg("x"), doc.Polynomial.Integrate.doc_1args)
      .def(
          "Integrate",
          [](const Polynomial& self, const Variable& var, double a, double b) {
            return self.Integrate(var, a, b);
          },
          py::arg("x"), py::arg("a"), py::arg("b"),
          doc.Polynomial.Integrate.doc_3args)
      .def("AddProduct", &Polynomial::AddProduct, py::arg("coeff"),
          py::arg("m"), doc.Polynomial.AddProduct.doc)
      .def("RemoveTermsWithSmallCoefficients",
          &Polynomial::RemoveTermsWithSmallCoefficients,
          py::arg("coefficient_tol"),
          doc.Polynomial.RemoveTermsWithSmallCoefficients.doc)
      .def("CoefficientsAlmostEqual", &Polynomial::CoefficientsAlmostEqual,
          py::arg("p"), py::arg("tolerance"),
          doc.Polynomial.CoefficientsAlmostEqual.doc)
      .def(py::self + py::self)
      .def(py::self + Monomial())
      .def(Monomial() + py::self)
      .def(py::self + double())
      .def(double() + py::self)
      .def(py::self + Variable())
      .def(Variable() + py::self)
      .def(py::self - py::self)
      .def(py::self - Monomial())
      .def(Monomial() - py::self)
      .def(py::self - double())
      .def(double() - py::self)
      .def(py::self - Variable())
      .def(Variable() - py::self)
      .def(py::self * py::self)
      .def(py::self * Monomial())
      .def(Monomial() * py::self)
      .def(py::self * double())
      .def(double() * py::self)
      .def(py::self * Variable())
      .def(Variable() * py::self)
      .def(-py::self)
      .def(py::self / double())
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
      .def(
          "Evaluate",
          [](const Polynomial& self, const Environment::map& env) {
            return self.Evaluate(Environment{env});
          },
          py::arg("env"), doc.Polynomial.Evaluate.doc)
      // TODO(Eric.Cousineau): add python binding for symbolic::Environment.
      .def(
          "EvaluatePartial",
          [](const Polynomial& self, const Environment::map& env) {
            return self.EvaluatePartial(Environment{env});
          },
          py::arg("env"), doc.Polynomial.EvaluatePartial.doc_1args)
      .def(
          "EvaluatePartial",
          [](const Polynomial& self, const Variable& var, double c) {
            return self.EvaluatePartial(var, c);
          },
          py::arg("var"), py::arg("c"),
          doc.Polynomial.EvaluatePartial.doc_2args)
      .def(
          "Jacobian",
          [](const Polynomial& p,
              const Eigen::Ref<const VectorX<Variable>>& vars) {
            return p.Jacobian(vars);
          },
          py::arg("vars"), doc.Polynomial.Jacobian.doc);

  m.def(
      "Evaluate",
      [](const MatrixX<Polynomial>& M, const Environment::map& env) {
        return Evaluate(M, Environment{env});
      },
      py::arg("m"), py::arg("env"), doc.Evaluate.doc_polynomial);

  m.def(
      "Jacobian",
      [](const Eigen::Ref<const VectorX<Polynomial>>& f,
          const Eigen::Ref<const VectorX<Variable>>& vars) {
        return Jacobian(f, vars);
      },
      py::arg("f"), py::arg("vars"), doc.Jacobian.doc_polynomial);

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

  // Bind the free functions in symbolic_decompose.h
  m  // BR
      .def(
          "DecomposeLinearExpressions",
          [](const Eigen::Ref<const VectorX<symbolic::Expression>>& expressions,
              const Eigen::Ref<const VectorX<Variable>>& vars) {
            Eigen::MatrixXd M(expressions.rows(), vars.rows());
            symbolic::DecomposeLinearExpressions(expressions, vars, &M);
            return M;
          },
          py::arg("expressions"), py::arg("vars"),
          doc.DecomposeLinearExpressions.doc)
      .def(
          "DecomposeAffineExpressions",
          [](const Eigen::Ref<const VectorX<symbolic::Expression>>& expressions,
              const Eigen::Ref<const VectorX<symbolic::Variable>>& vars) {
            Eigen::MatrixXd M(expressions.rows(), vars.rows());
            Eigen::VectorXd v(expressions.rows());
            symbolic::DecomposeAffineExpressions(expressions, vars, &M, &v);
            return std::make_pair(M, v);
          },
          py::arg("expressions"), py::arg("vars"),
          doc.DecomposeAffineExpressions.doc_4args_expressions_vars_M_v)
      .def("ExtractVariablesFromExpression",
          &symbolic::ExtractVariablesFromExpression, py::arg("e"),
          doc.ExtractVariablesFromExpression.doc)
      .def(
          "DecomposeQuadraticPolynomial",
          [](const symbolic::Polynomial& poly,
              const std::unordered_map<symbolic::Variable::Id, int>&
                  map_var_to_index) {
            const int num_vars = map_var_to_index.size();
            Eigen::MatrixXd Q(num_vars, num_vars);
            Eigen::VectorXd b(num_vars);
            double c;
            symbolic::DecomposeQuadraticPolynomial(
                poly, map_var_to_index, &Q, &b, &c);
            return std::make_tuple(Q, b, c);
          },
          py::arg("poly"), py::arg("map_var_to_index"),
          doc.DecomposeQuadraticPolynomial.doc)
      .def(
          "DecomposeAffineExpressions",
          [](const Eigen::Ref<const VectorX<symbolic::Expression>>& v) {
            Eigen::MatrixXd A;
            Eigen::VectorXd b;
            VectorX<Variable> vars;
            symbolic::DecomposeAffineExpressions(v, &A, &b, &vars);
            return std::make_tuple(A, b, vars);
          },
          py::arg("v"), doc.DecomposeAffineExpressions.doc_4args_v_A_b_vars)
      .def(
          "DecomposeAffineExpression",
          [](const symbolic::Expression& e,
              const std::unordered_map<symbolic::Variable::Id, int>&
                  map_var_to_index) {
            Eigen::RowVectorXd coeffs(map_var_to_index.size());
            double constant_term;
            symbolic::DecomposeAffineExpression(
                e, map_var_to_index, coeffs, &constant_term);
            return std::make_pair(coeffs, constant_term);
          },
          py::arg("e"), py::arg("map_var_to_index"),
          doc.DecomposeAffineExpression.doc)
      .def("DecomposeLumpedParameters", &DecomposeLumpedParameters,
          py::arg("f"), py::arg("parameters"),
          doc.DecomposeLumpedParameters.doc);

  // NOLINTNEXTLINE(readability/fn_size)
}
}  // namespace pydrake
}  // namespace drake

#pragma GCC diagnostic pop
