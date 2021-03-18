#include "drake/common/symbolic.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py = pybind11;
void apb11_pydrake_Expression_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::symbolic;

  py::class_<Expression> PyExpression(
      m, "Expression",
      R"""(/** Represents a symbolic form of an expression. 
 
Its syntax tree is as follows: 
 
@verbatim 
    E := Var | Constant | E + ... + E | E * ... * E | E / E | log(E) 
       | abs(E) | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E) 
       | asin(E) | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E) 
       | min(E, E) | max(E, E) | ceil(E) | floor(E) | if_then_else(F, E, E) 
       | NaN | uninterpreted_function(name, {v_1, ..., v_n}) 
@endverbatim 
 
In the implementation, Expression is a simple wrapper including a shared pointer 
to ExpressionCell class which is a super-class of different kinds of symbolic 
expressions (i.e. ExpressionAdd, ExpressionMul, ExpressionLog, 
ExpressionSin). Note that it includes a shared pointer, not a unique pointer, to 
allow sharing sub-expressions. 
 
@note The sharing of sub-expressions is not yet implemented. 
 
@note -E is represented as -1 * E internally. 
 
@note A subtraction E1 - E2 is represented as E1 + (-1 * E2) internally. 
 
The following simple simplifications are implemented: 
@verbatim 
    E + 0             ->  E 
    0 + E             ->  E 
    E - 0             ->  E 
    E - E             ->  0 
    E * 1             ->  E 
    1 * E             ->  E 
    E * 0             ->  0 
    0 * E             ->  0 
    E / 1             ->  E 
    E / E             ->  1 
    pow(E, 0)         ->  1 
    pow(E, 1)         ->  E 
    E * E             ->  E^2 (= pow(E, 2)) 
    sqrt(E * E)       ->  |E| (= abs(E)) 
    sqrt(E) * sqrt(E) -> E 
@endverbatim 
 
Constant folding is implemented: 
@verbatim 
    E(c1) + E(c2)  ->  E(c1 + c2)    // c1, c2 are constants 
    E(c1) - E(c2)  ->  E(c1 - c2) 
    E(c1) * E(c2)  ->  E(c1 * c2) 
    E(c1) / E(c2)  ->  E(c1 / c2) 
    f(E(c))        ->  E(f(c))       // c is a constant, f is a math function 
@endverbatim 
 
For the math functions which are only defined over a restricted domain (namely, 
log, sqrt, pow, asin, acos), we check the domain of argument(s), and throw 
std::domain_error exception if a function is not well-defined for a given 
argument(s). 
 
Relational operators over expressions (==, !=, <, >, <=, >=) return 
symbolic::Formula instead of bool. Those operations are declared in 
symbolic_formula.h file. To check structural equality between two expressions a 
separate function, Expression::EqualTo, is provided. 
 
Regarding NaN, we have the following rules: 
 1. NaN values are extremely rare during typical computations. Because they are 
    difficult to handle symbolically, we will round that up to "must never 
    occur". We allow the user to form ExpressionNaN cells in a symbolic 
    tree. For example, the user can initialize an Expression to NaN and then 
    overwrite it later. However, evaluating a tree that has NaN in its evaluated 
    sub-trees is an error (see rule (3) below). 
 2. It's still valid for code to check `isnan` in order to fail-fast. So we 
    provide isnan(const Expression&) for the common case of non-NaN value 
    returning False. This way, code can fail-fast with double yet still compile 
    with Expression. 
 3. If there are expressions that embed separate cases (`if_then_else`), some of 
    the sub-expressions may be not used in evaluation when they are in the 
    not-taken case (for NaN reasons or any other reason). Bad values within 
    those not-taken branches does not cause exceptions. 
 4. The isnan check is different than if_then_else. In the latter, the 
    ExpressionNaN is within a dead sub-expression branch. In the former, it 
    appears in an evaluated trunk. That goes against rule (1) where a NaN 
    anywhere in a computation (other than dead code) is an error. 
 
symbolic::Expression can be used as a scalar type of Eigen types. 
*/)""");

  PyExpression.def(py::init<Expression const &>(), py::arg("arg0"))
      .def(py::init<>())
      .def(py::init<double>(), py::arg("d"))
      .def(py::init<Variable const &>(), py::arg("var"))
      .def_static("DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE",
                  static_cast<void (*)()>(
                      &Expression::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
      .def(
          "Differentiate",
          static_cast<Expression (Expression::*)(Variable const &) const>(
              &Expression::Differentiate),
          py::arg("x"),
          R"""(/** Differentiates this symbolic expression with respect to the variable @p 
 * var. 
 * @throws std::runtime_error if it is not differentiable. 
 */)""")
      .def_static("E", static_cast<Expression (*)()>(&Expression::E),
                  R"""(/** Return e, the base of natural logarithms. */)""")
      .def("EqualTo",
           static_cast<bool (Expression::*)(Expression const &) const>(
               &Expression::EqualTo),
           py::arg("e"),
           R"""(/** Checks structural equality. 
 * 
 * Two expressions e1 and e2 are structurally equal when they have the same 
 * internal AST(abstract-syntax tree) representation. Please note that we can 
 * have two computationally (or extensionally) equivalent expressions which 
 * are not structurally equal. For example, consider: 
 * 
 *    e1 = 2 * (x + y) 
 *    e2 = 2x + 2y 
 * 
 * Obviously, we know that e1 and e2 are evaluated to the same value for all 
 * assignments to x and y. However, e1 and e2 are not structurally equal by 
 * the definition. Note that e1 is a multiplication expression 
 * (is_multiplication(e1) is true) while e2 is an addition expression 
 * (is_addition(e2) is true). 
 * 
 * One main reason we use structural equality in EqualTo is due to 
 * Richardson's Theorem. It states that checking ∀x. E(x) = F(x) is 
 * undecidable when we allow sin, asin, log, exp in E and F. Read 
 * https://en.wikipedia.org/wiki/Richardson%27s_theorem for details. 
 * 
 * Note that for polynomial cases, you can use Expand method and check if two 
 * polynomial expressions p1 and p2 are computationally equal. To do so, you 
 * check the following: 
 * 
 *     (p1.Expand() - p2.Expand()).EqualTo(0). 
 */)""")
      .def(
          "Evaluate",
          static_cast<double (Expression::*)(Environment const &,
                                             ::drake::RandomGenerator *) const>(
              &Expression::Evaluate),
          py::arg("env") = (Environment)drake::symbolic::Environment{},
          py::arg("random_generator") = (::drake::RandomGenerator *)nullptr,
          R"""(/** Evaluates using a given environment (by default, an empty environment) and 
 * a random number generator. If there is a random variable in this expression 
 * which is unassigned in @p env, this method uses @p random_generator to 
 * sample a value and use the value to substitute all occurrences of the 
 * variable in this expression. 
 * 
 * @throws std::runtime_error if there exists a non-random variable in this 
 *                            expression whose assignment is not provided by 
 *                            @p env. 
 * @throws std::runtime_error if an unassigned random variable is detected 
 *                            while @p random_generator is `nullptr`. 
 * @throws std::runtime_error if NaN is detected during evaluation. 
 */)""")
      .def(
          "Evaluate",
          static_cast<double (Expression::*)(::drake::RandomGenerator *) const>(
              &Expression::Evaluate),
          py::arg("random_generator"),
          R"""(/** Evaluates using an empty environment and a random number generator. It 
 * uses @p random_generator to sample values for the random variables in this 
 * expression. 
 * 
 * See the above overload for the exceptions that it might throw. 
 */)""")
      .def("EvaluatePartial",
           static_cast<Expression (Expression::*)(Environment const &) const>(
               &Expression::EvaluatePartial),
           py::arg("env"),
           R"""(/** Partially evaluates this expression using an environment @p 
 * env. Internally, this method promotes @p env into a substitution 
 * (Variable → Expression) and call Evaluate::Substitute with it. 
 * 
 * @throws std::runtime_error if NaN is detected during evaluation. 
 */)""")
      .def(
          "Expand",
          static_cast<Expression (Expression::*)() const>(&Expression::Expand),
          R"""(/** Expands out products and positive integer powers in expression. For 
 * example, `(x + 1) * (x - 1)` is expanded to `x^2 - 1` and `(x + y)^2` is 
 * expanded to `x^2 + 2xy + y^2`. Note that Expand applies recursively to 
 * sub-expressions. For instance, `sin(2 * (x + y))` is expanded to `sin(2x + 
 * 2y)`. It also simplifies "division by constant" cases. See 
 * "drake/common/test/symbolic_expansion_test.cc" to find the examples. 
 * 
 * @throws std::runtime_error if NaN is detected during expansion. 
 */)""")
      .def("GetVariables",
           static_cast<Variables (Expression::*)() const>(
               &Expression::GetVariables),
           R"""(/** Collects variables in expression. */)""")
      .def("Jacobian",
           [](Expression &self,
              ::Eigen::Ref<const Eigen::Matrix<drake::symbolic::Variable, -1, 1,
                                               0, -1, 1>,
                           0, Eigen::InnerStride<1>> const &vars) {
             return self.Jacobian(vars);
           })
      .def("Less",
           static_cast<bool (Expression::*)(Expression const &) const>(
               &Expression::Less),
           py::arg("e"),
           R"""(/** Provides lexicographical ordering between expressions. 
    This function is used as a compare function in map<Expression> and 
    set<Expression> via std::less<drake::symbolic::Expression>. */)""")
      .def_static("NaN", static_cast<Expression (*)()>(&Expression::NaN),
                  R"""(/** Returns NaN (Not-a-Number). */)""")
      .def_static("One", static_cast<Expression (*)()>(&Expression::One),
                  R"""(/** Returns one. */)""")
      .def_static(
          "Pi", static_cast<Expression (*)()>(&Expression::Pi),
          R"""(/** Returns Pi, the ratio of a circle’s circumference to its diameter. */)""")
      .def(
          "Substitute",
          static_cast<Expression (Expression::*)(Variable const &,
                                                 Expression const &) const>(
              &Expression::Substitute),
          py::arg("var"), py::arg("e"),
          R"""(/** Returns a copy of this expression replacing all occurrences of @p var 
 * with @p e. 
 * @throws std::runtime_error if NaN is detected during substitution. 
 */)""")
      .def(
          "Substitute",
          static_cast<Expression (Expression::*)(Substitution const &) const>(
              &Expression::Substitute),
          py::arg("s"),
          R"""(/** Returns a copy of this expression replacing all occurrences of the 
 * variables in @p s with corresponding expressions in @p s. Note that the 
 * substitutions occur simultaneously. For example, (x / y).Substitute({{x, 
 * y}, {y, x}}) gets (y / x). 
 * @throws std::runtime_error if NaN is detected during substitution. 
 */)""")
      .def_static("Zero", static_cast<Expression (*)()>(&Expression::Zero),
                  R"""(/** Returns zero. */)""")
      .def("get_kind",
           static_cast<ExpressionKind (Expression::*)() const>(
               &Expression::get_kind),
           R"""(/** Returns expression kind. */)""")
      .def("is_expanded",
           static_cast<bool (Expression::*)() const>(&Expression::is_expanded),
           R"""(/** Returns true if this symbolic expression is already 
 * expanded. Expression::Expand() uses this flag to avoid calling 
 * ExpressionCell::Expand() on an pre-expanded expressions. 
 * Expression::Expand() also sets this flag before returning the result. 
 * 
 * @note This check is conservative in that `false` does not always indicate 
 * that the expression is not expanded. This is because exact checks can be 
 * costly and we want to avoid the exact check at the construction time. 
 */)""")
      .def(
          "is_polynomial",
          static_cast<bool (Expression::*)() const>(&Expression::is_polynomial),
          R"""(/** Checks if this symbolic expression is convertible to Polynomial. */)""")
      .def("to_string",
           static_cast<::std::string (Expression::*)() const>(
               &Expression::to_string),
           R"""(/** Returns string representation of Expression. */)""")

      .def(
          "__mul__",
          +[](Expression lhs, Expression const &rhs) { return lhs * rhs; })
      .def(
          "__imul__",
          +[](Expression &lhs, Expression const &rhs) { return lhs *= rhs; })
      .def(
          "__add__",
          +[](Expression lhs, Expression const &rhs) { return lhs + rhs; })
      .def(
          "__add__", +[](Expression const &e) { return e; })
      .def("__add__",
           static_cast<Expression &(Expression::*)()>(&Expression::operator++),
           R"""(/** Provides prefix increment operator (i.e. ++x). */)""")
      .def(
          "__add__",
          static_cast<Expression (Expression::*)(int)>(&Expression::operator++),
          py::arg("arg0"),
          R"""(/** Provides postfix increment operator (i.e. x++). */)""")
      .def(
          "__iadd__",
          +[](Expression &lhs, Expression const &rhs) { return lhs += rhs; })
      .def(
          "__sub__",
          +[](Expression lhs, Expression const &rhs) { return lhs - rhs; })
      .def(
          "__sub__", +[](Expression const &e) { return e; })
      .def("__sub__",
           static_cast<Expression &(Expression::*)()>(&Expression::operator--),
           R"""(/** Provides prefix decrement operator (i.e. --x). */)""")
      .def(
          "__sub__",
          static_cast<Expression (Expression::*)(int)>(&Expression::operator--),
          py::arg("arg0"),
          R"""(/** Provides postfix decrement operator (i.e. x--). */)""")
      .def(
          "__isub__",
          +[](Expression &lhs, Expression const &rhs) { return lhs -= rhs; })
      .def(
          "__truediv__",
          +[](Expression lhs, Expression const &rhs) { return lhs / rhs; })
      .def(
          "__itruediv__",
          +[](Expression &lhs, Expression const &rhs) { return lhs /= rhs; })
      .def(
          "__str__", +[](Expression const &e) {
            std::ostringstream oss;
            oss << e;
            std::string s(oss.str());

            return s;
          });
}
