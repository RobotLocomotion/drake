#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/common/symbolic/expression/all.h"
// #include "drake/common/symbolic/expression/boxed_cell.h"
// #include "drake/common/symbolic/expression/environment.h"
// #include "drake/common/symbolic/expression/expression.h"
// #include "drake/common/symbolic/expression/expression_cell.h"
// #include "drake/common/symbolic/expression/expression_kind.h"
// #include "drake/common/symbolic/expression/expression_visitor.h"
// #include "drake/common/symbolic/expression/formula.h"
// #include "drake/common/symbolic/expression/formula_cell.h"
// #include "drake/common/symbolic/expression/formula_visitor.h"
// #include "drake/common/symbolic/expression/ldlt.h"
// #include "drake/common/symbolic/expression/variable.h"
// #include "drake/common/symbolic/expression/variables.h"

// Symbol: pydrake_doc_common_symbolic_expression
constexpr struct /* pydrake_doc_common_symbolic_expression */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::ExtractDoubleOrThrow
    struct /* ExtractDoubleOrThrow */ {
      // Source: drake/common/symbolic/expression/expression.h
      const char* doc_1args_e =
R"""(Returns the symbolic expression's value() as a double.

Raises:
    RuntimeError if it is not possible to evaluate the symbolic
    expression with an empty environment.)""";
      // Source: drake/common/symbolic/expression/expression.h
      const char* doc_1args_constEigenMatrixBase =
R"""(Returns ``matrix`` as an Eigen∷Matrix<double, ...> with the same size
allocation as ``matrix``. Calls ExtractDoubleOrThrow on each element
of the matrix, and therefore throws if any one of the extractions
fail.)""";
    } ExtractDoubleOrThrow;
    // Symbol: drake::assert
    struct /* assert */ {
      // Symbol: drake::assert::ConditionTraits
      struct /* ConditionTraits */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""()""";
        // Symbol: drake::assert::ConditionTraits::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""()""";
        } Evaluate;
      } ConditionTraits;
    } assert;
    // Symbol: drake::cond
    struct /* cond */ {
      // Source: drake/common/symbolic/expression/expression.h
      const char* doc =
R"""(Provides specialization of ``cond`` function defined in
drake/common/cond.h file. This specialization is required to handle
``double`` to ``symbolic``∷Expression conversion so that we can write
one such as ``cond(x > 0.0, 1.0, -1.0)``.)""";
    } cond;
    // Symbol: drake::dummy_value
    struct /* dummy_value */ {
      // Source: drake/common/symbolic/expression/expression.h
      const char* doc = R"""(Specializes common/dummy_value.h.)""";
      // Symbol: drake::dummy_value::get
      struct /* get */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } get;
    } dummy_value;
    // Symbol: drake::is_eigen_nonvector_expression_double_pair
    struct /* is_eigen_nonvector_expression_double_pair */ {
      // Source: drake/common/symbolic/expression/expression.h
      const char* doc = R"""()""";
    } is_eigen_nonvector_expression_double_pair;
    // Symbol: drake::is_eigen_vector_expression_double_pair
    struct /* is_eigen_vector_expression_double_pair */ {
      // Source: drake/common/symbolic/expression/expression.h
      const char* doc = R"""()""";
    } is_eigen_vector_expression_double_pair;
    // Symbol: drake::symbolic
    struct /* symbolic */ {
      // Symbol: drake::symbolic::BinaryExpressionCell
      struct /* BinaryExpressionCell */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Represents the base class for binary expressions.)""";
        // Symbol: drake::symbolic::BinaryExpressionCell::BinaryExpressionCell
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs BinaryExpressionCell of kind ``k`` with ``e1``, ``e2``,
``is_poly``, and ``is_expanded``.)""";
        } ctor;
        // Symbol: drake::symbolic::BinaryExpressionCell::DoEvaluate
        struct /* DoEvaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns the evaluation result f(``v1``, ``v2`` ).)""";
        } DoEvaluate;
        // Symbol: drake::symbolic::BinaryExpressionCell::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::BinaryExpressionCell::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::BinaryExpressionCell::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::BinaryExpressionCell::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::BinaryExpressionCell::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::BinaryExpressionCell::get_first_argument
        struct /* get_first_argument */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the first argument.)""";
        } get_first_argument;
        // Symbol: drake::symbolic::BinaryExpressionCell::get_second_argument
        struct /* get_second_argument */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the second argument.)""";
        } get_second_argument;
      } BinaryExpressionCell;
      // Symbol: drake::symbolic::CheckStructuralEquality
      struct /* CheckStructuralEquality */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Checks if two Eigen∷Matrix<Expression> ``m1`` and ``m2`` are
structurally equal. That is, it returns true if and only if ``m1(i,
j)`` is structurally equal to ``m2(i, j)`` for all ``i``, `j`.)""";
      } CheckStructuralEquality;
      // Symbol: drake::symbolic::Environment
      struct /* Environment */ {
        // Source: drake/common/symbolic/expression/environment.h
        const char* doc =
R"""(Represents a symbolic environment (mapping from a variable to a
value).

This class is used when we evaluate symbolic expressions or formulas
which include unquantified (free) variables. Here are examples:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const Variable var_x{"x"};
      const Variable var_y{"y"};
      const Expression x{var_x};
      const Expression y{var_x};
      const Expression e1{x + y};
      const Expression e2{x - y};
      const Formula f{e1 > e2};
    
      // env maps var_x to 2.0 and var_y to 3.0
      const Environment env{{var_x, 2.0}, {var_y, 3.0}};
    
      const double res1 = e1.Evaluate(env);  // x + y => 2.0 + 3.0 =>  5.0
      const double res2 = e2.Evaluate(env);  // x - y => 2.0 - 3.0 => -1.0
      const bool res = f.Evaluate(env);  // x + y > x - y => 5.0 >= -1.0 => True

.. raw:: html

    </details>)""";
        // Symbol: drake::symbolic::Environment::Environment
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_0args = R"""(Default constructor.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_1args_init =
R"""(List constructor. Constructs an environment from a list of (Variable *
double).

Raises:
    RuntimeError if ``init`` include a dummy variable or a NaN value.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_1args_vars =
R"""(List constructor. Constructs an environment from a list of Variable.
Initializes the variables with 0.0.

Raises:
    RuntimeError if ``vars`` include a dummy variable.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_1args_m =
R"""(Constructs an environment from ``m`` (of ``map`` type, which is
``std∷unordered_map``).

Raises:
    RuntimeError if ``m`` include a dummy variable or a NaN value.)""";
        } ctor;
        // Symbol: drake::symbolic::Environment::begin
        struct /* begin */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_0args_nonconst = R"""(Returns an iterator to the beginning.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_0args_const =
R"""(Returns a const iterator to the beginning.)""";
        } begin;
        // Symbol: drake::symbolic::Environment::cbegin
        struct /* cbegin */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc =
R"""(Returns a const iterator to the beginning.)""";
        } cbegin;
        // Symbol: drake::symbolic::Environment::cend
        struct /* cend */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(Returns a const iterator to the end.)""";
        } cend;
        // Symbol: drake::symbolic::Environment::const_iterator
        struct /* const_iterator */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""()""";
        } const_iterator;
        // Symbol: drake::symbolic::Environment::domain
        struct /* domain */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(Returns the domain of this environment.)""";
        } domain;
        // Symbol: drake::symbolic::Environment::empty
        struct /* empty */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(Checks whether the container is empty.)""";
        } empty;
        // Symbol: drake::symbolic::Environment::end
        struct /* end */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_0args_nonconst = R"""(Returns an iterator to the end.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_0args_const = R"""(Returns a const iterator to the end.)""";
        } end;
        // Symbol: drake::symbolic::Environment::find
        struct /* find */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(Finds element with specific key.)""";
        } find;
        // Symbol: drake::symbolic::Environment::insert
        struct /* insert */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_2args_key_elem =
R"""(Inserts a pair (``key``, ``elem)`` if this environment doesn't contain
``key``. Similar to insert function in map, if the key already exists
in this environment, then calling insert(key, elem) doesn't change the
existing key-value in this environment.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_2args_keys_elements =
R"""(Given a matrix of symbolic variables ``keys`` and a matrix of values
``elements``, inserts each pair (keys(i, j), elements(i, j)) into the
environment if this environment doesn't contain keys(i, j) . Similar
to insert function in map, if keys(i, j) already exists in this
environment, then this function doesn't change the its existing value
in this environment.

Raises:
    RuntimeError if the size of ``keys`` is different from the size of
    ``elements``.)""";
        } insert;
        // Symbol: drake::symbolic::Environment::iterator
        struct /* iterator */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""()""";
        } iterator;
        // Symbol: drake::symbolic::Environment::key_type
        struct /* key_type */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""()""";
        } key_type;
        // Symbol: drake::symbolic::Environment::map
        struct /* map */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""()""";
        } map;
        // Symbol: drake::symbolic::Environment::mapped_type
        struct /* mapped_type */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""()""";
        } mapped_type;
        // Symbol: drake::symbolic::Environment::operator[]
        struct /* operator_array */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_1args_key_nonconst =
R"""(Returns a reference to the value that is mapped to a key equivalent to
``key``, performing an insertion if such key does not already exist.)""";
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc_1args_key_const =
R"""(As above, but returns a constref and does not perform an insertion
(throwing a runtime error instead) if the key does not exist.)""";
        } operator_array;
        // Symbol: drake::symbolic::Environment::size
        struct /* size */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(Returns the number of elements.)""";
        } size;
        // Symbol: drake::symbolic::Environment::to_string
        struct /* to_string */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(Returns string representation.)""";
        } to_string;
        // Symbol: drake::symbolic::Environment::value_type
        struct /* value_type */ {
          // Source: drake/common/symbolic/expression/environment.h
          const char* doc = R"""(std∷pair<key_type, mapped_type>)""";
        } value_type;
      } Environment;
      // Symbol: drake::symbolic::Evaluate
      struct /* Evaluate */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_expression =
R"""(Evaluates a symbolic matrix ``m`` using ``env`` and
``random_generator``.

If there is a random variable in ``m`` which is unassigned in ``env``,
this function uses ``random_generator`` to sample a value and use the
value to substitute all occurrences of the random variable in ``m``.

Returns:
    a matrix of double whose size is the size of ``m``.

Raises:
    RuntimeError if NaN is detected during evaluation.

Raises:
    RuntimeError if ``m`` includes unassigned random variables but
    ``random_generator`` is ``nullptr``.)""";
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Evaluates ``m`` using a given environment (by default, an empty
environment).

Raises:
    RuntimeError if there exists a variable in ``m`` whose value is
    not provided by ``env``.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
      } Evaluate;
      // Symbol: drake::symbolic::Expression
      struct /* Expression */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Represents a symbolic form of an expression.

Its syntax tree is as follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    E := Var | Constant | E + ... + E | E * ... * E | E / E | log(E)
    | abs(E) | exp(E) | sqrt(E) | pow(E, E) | sin(E) | cos(E) | tan(E)
    | asin(E) | acos(E) | atan(E) | atan2(E, E) | sinh(E) | cosh(E) | tanh(E)
    | min(E, E) | max(E, E) | ceil(E) | floor(E) | if_then_else(F, E, E)
    | NaN | uninterpreted_function(name, {v_1, ..., v_n})

.. raw:: html

    </details>

In the implementation, Expression directly stores Constant values
inline, but in all other cases stores a shared pointer to a const
ExpressionCell class that is a super-class of different kinds of
symbolic expressions (i.e., ExpressionAdd, ExpressionMul,
ExpressionLog, ExpressionSin), which makes it efficient to copy, move,
and assign to an Expression.

Note:
    -E is represented as -1 * E internally.

Note:
    A subtraction E1 - E2 is represented as E1 + (-1 * E2) internally.

The following simple simplifications are implemented:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

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

.. raw:: html

    </details>

Constant folding is implemented:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    E(c1) + E(c2)  ->  E(c1 + c2)    // c1, c2 are constants
    E(c1) - E(c2)  ->  E(c1 - c2)
    E(c1) * E(c2)  ->  E(c1 * c2)
    E(c1) / E(c2)  ->  E(c1 / c2)
    f(E(c))        ->  E(f(c))       // c is a constant, f is a math function

.. raw:: html

    </details>

For the math functions which are only defined over a restricted domain
(namely, log, sqrt, pow, asin, acos), we check the domain of
argument(s), and throw ValueError exception if a function is not
well-defined for a given argument(s).

Relational operators over expressions (==, !=, <, >, <=, >=) return
symbolic∷Formula instead of bool. Those operations are declared in
formula.h file. To check structural equality between two expressions a
separate function, Expression∷EqualTo, is provided.

Regarding the arithmetic of an Expression when operating on NaNs, we
have the following rules: 1. NaN values are extremely rare during
typical computations. Because they are difficult to handle
symbolically, we will round that up to "must never occur". We allow
the user to form ExpressionNaN cells in a symbolic tree. For example,
the user can initialize an Expression to NaN and then overwrite it
later. However, evaluating a tree that has NaN in its evaluated
sub-trees is an error (see rule (3) below). 2. It's still valid for
code to check ``isnan`` in order to fail-fast. So we provide
isnan(const Expression&) for the common case of non-NaN value
returning False. This way, code can fail-fast with double yet still
compile with Expression. 3. If there are expressions that embed
separate cases (``if_then_else``), some of the sub-expressions may be
not used in evaluation when they are in the not-taken case (for NaN
reasons or any other reason). Bad values within those not-taken
branches does not cause exceptions. 4. The isnan check is different
than if_then_else. In the latter, the ExpressionNaN is within a dead
sub-expression branch. In the former, it appears in an evaluated
trunk. That goes against rule (1) where a NaN anywhere in a
computation (other than dead code) is an error.

note for Drake developers: under the hood of Expression, we have an
internal∷BoxedCell helper class that uses NaN for pointer tagging;
that's a distinct concept from the Expression∷NaN() rules enumerated
just above.

symbolic∷Expression can be used as a scalar type of Eigen types.)""";
        // Symbol: drake::symbolic::Expression::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Differentiates this symbolic expression with respect to the variable
``var``.

Raises:
    RuntimeError if it is not differentiable.)""";
        } Differentiate;
        // Symbol: drake::symbolic::Expression::E
        struct /* E */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Return e, the base of natural logarithms.)""";
        } E;
        // Symbol: drake::symbolic::Expression::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Checks structural equality.

Two expressions e1 and e2 are structurally equal when they have the
same internal AST(abstract-syntax tree) representation. Please note
that we can have two computationally (or extensionally) equivalent
expressions which are not structurally equal. For example, consider:

e1 = 2 * (x + y) e2 = 2x + 2y

Obviously, we know that e1 and e2 are evaluated to the same value for
all assignments to x and y. However, e1 and e2 are not structurally
equal by the definition. Note that e1 is a multiplication expression
(is_multiplication(e1) is true) while e2 is an addition expression
(is_addition(e2) is true).

One main reason we use structural equality in EqualTo is due to
Richardson's Theorem. It states that checking ∀x. E(x) = F(x) is
undecidable when we allow sin, asin, log, exp in E and F. Read
https://en.wikipedia.org/wiki/Richardson%27s_theorem for details.

Note that for polynomial cases, you can use Expand method and check if
two polynomial expressions p1 and p2 are computationally equal. To do
so, you check the following:

p1.Expand().EqualTo(p2.Expand()))""";
        } EqualTo;
        // Symbol: drake::symbolic::Expression::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_2args =
R"""(Evaluates using a given environment (by default, an empty environment)
and a random number generator. If there is a random variable in this
expression which is unassigned in ``env``, this method uses
``random_generator`` to sample a value and use the value to substitute
all occurrences of the variable in this expression.

Raises:
    RuntimeError if there exists a non-random variable in this
    expression whose assignment is not provided by ``env``.

Raises:
    RuntimeError if an unassigned random variable is detected while
    ``random_generator`` is ``nullptr``.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_1args =
R"""(Evaluates using an empty environment and a random number generator. It
uses ``random_generator`` to sample values for the random variables in
this expression.

See the above overload for the exceptions that it might throw.)""";
        } Evaluate;
        // Symbol: drake::symbolic::Expression::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Partially evaluates this expression using an environment ``env``.
Internally, this method promotes ``env`` into a substitution (Variable
→ Expression) and call Evaluate∷Substitute with it.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::Expression::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Expands out products and positive integer powers in expression. For
example, ``(x + 1) * (x - 1)`` is expanded to ``x^2 - 1`` and ``(x +
y)^2`` is expanded to ``x^2 + 2xy + y^2``. Note that Expand applies
recursively to sub-expressions. For instance, ``sin(2 * (x + y))`` is
expanded to ``sin(2x + 2y)``. It also simplifies "division by
constant" cases. See "drake/common/test/symbolic_expansion_test.cc" to
find the examples.

Raises:
    RuntimeError if NaN is detected during expansion.)""";
        } Expand;
        // Symbol: drake::symbolic::Expression::Expression
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_0args =
R"""(Default constructor. It constructs Zero().)""";
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_1args_constant = R"""(Constructs a constant.)""";
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_1args_var =
R"""(Constructs an expression from ``var``.

Precondition:
    ``var`` is not a BOOLEAN variable.)""";
        } ctor;
        // Symbol: drake::symbolic::Expression::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Same as GetVariables(); we provide this overload for compatibility
with Formula.)""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::Expression::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc = R"""(Collects variables in expression.)""";
        } GetVariables;
        // Symbol: drake::symbolic::Expression::Jacobian
        struct /* Jacobian */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Let ``f`` be this Expression, computes a row vector of derivatives,
``[∂f/∂vars(0), ... , ∂f/∂vars(n-1)]`` with respect to the variables
``vars``.)""";
        } Jacobian;
        // Symbol: drake::symbolic::Expression::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Provides lexicographical ordering between expressions. This function
is used as a compare function in map<Expression> and set<Expression>
via std∷less<drake∷symbolic∷Expression>.)""";
        } Less;
        // Symbol: drake::symbolic::Expression::NaN
        struct /* NaN */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc = R"""(Returns NaN (Not-a-Number).)""";
        } NaN;
        // Symbol: drake::symbolic::Expression::One
        struct /* One */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc = R"""(Returns one.)""";
        } One;
        // Symbol: drake::symbolic::Expression::Pi
        struct /* Pi */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Returns Pi, the ratio of a circle’s circumference to its diameter.)""";
        } Pi;
        // Symbol: drake::symbolic::Expression::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_2args =
R"""(Returns a copy of this expression replacing all occurrences of ``var``
with ``e``.

Raises:
    RuntimeError if NaN is detected during substitution.)""";
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_1args =
R"""(Returns a copy of this expression replacing all occurrences of the
variables in ``s`` with corresponding expressions in ``s``. Note that
the substitutions occur simultaneously. For example, (x /
y).Substitute({{x, y}, {y, x}}) gets (y / x).

Raises:
    RuntimeError if NaN is detected during substitution.)""";
        } Substitute;
        // Symbol: drake::symbolic::Expression::Zero
        struct /* Zero */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc = R"""(Returns zero.)""";
        } Zero;
        // Symbol: drake::symbolic::Expression::get_kind
        struct /* get_kind */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc = R"""(Returns expression kind.)""";
        } get_kind;
        // Symbol: drake::symbolic::Expression::is_expanded
        struct /* is_expanded */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Returns true if this symbolic expression is already expanded.
Expression∷Expand() uses this flag to avoid calling
ExpressionCell∷Expand() on an pre-expanded expressions.
Expression∷Expand() also sets this flag before returning the result.

Note:
    This check is conservative in that ``False`` does not always
    indicate that the expression is not expanded. This is because
    exact checks can be costly and we want to avoid the exact check at
    the construction time.)""";
        } is_expanded;
        // Symbol: drake::symbolic::Expression::is_polynomial
        struct /* is_polynomial */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Checks if this symbolic expression is convertible to Polynomial.)""";
        } is_polynomial;
        // Symbol: drake::symbolic::Expression::operator++
        struct /* operator_inc */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_0args =
R"""(Provides prefix increment operator (i.e. ++x).)""";
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_1args =
R"""(Provides postfix increment operator (i.e. x++).)""";
        } operator_inc;
        // Symbol: drake::symbolic::Expression::operator--
        struct /* operator_dec */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_0args =
R"""(Provides prefix decrement operator (i.e. --x).)""";
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc_1args =
R"""(Provides postfix decrement operator (i.e. x--).)""";
        } operator_dec;
        // Symbol: drake::symbolic::Expression::to_string
        struct /* to_string */ {
          // Source: drake/common/symbolic/expression/expression.h
          const char* doc =
R"""(Returns string representation of Expression.)""";
        } to_string;
      } Expression;
      // Symbol: drake::symbolic::ExpressionAbs
      struct /* ExpressionAbs */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing absolute value function.)""";
        // Symbol: drake::symbolic::ExpressionAbs::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionAbs::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionAbs::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionAbs::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionAbs::ExpressionAbs
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAbs::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionAbs;
      // Symbol: drake::symbolic::ExpressionAcos
      struct /* ExpressionAcos */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing arccosine function.)""";
        // Symbol: drake::symbolic::ExpressionAcos::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionAcos::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionAcos::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionAcos::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionAcos::ExpressionAcos
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAcos::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionAcos;
      // Symbol: drake::symbolic::ExpressionAdd
      struct /* ExpressionAdd */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing an addition which is a sum of
products.

.. math:: c_0 + \sum c_i * e_i

where :math:`c_i` is a constant and :math:`e_i` is a symbolic
expression.

Internally this class maintains a member variable ``constant_`` to
represent :math:`c_0` and another member variable
``expr_to_coeff_map_`` to represent a mapping from an expression
:math:`e_i` to its coefficient :math:`c_i` of double.)""";
        // Symbol: drake::symbolic::ExpressionAdd::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionAdd::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionAdd::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionAdd::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionAdd::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionAdd::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionAdd::ExpressionAdd
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs ExpressionAdd from ``constant_term`` and
``term_to_coeff_map``.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAdd::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionAdd::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionAdd::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::ExpressionAdd::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::ExpressionAdd::get_constant
        struct /* get_constant */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the constant.)""";
        } get_constant;
        // Symbol: drake::symbolic::ExpressionAdd::get_expr_to_coeff_map
        struct /* get_expr_to_coeff_map */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns map from an expression to its coefficient.)""";
        } get_expr_to_coeff_map;
      } ExpressionAdd;
      // Symbol: drake::symbolic::ExpressionAddFactory
      struct /* ExpressionAddFactory */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Factory class to help build ExpressionAdd expressions.)""";
        // Symbol: drake::symbolic::ExpressionAddFactory::Add
        struct /* Add */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Adds ExpressionAdd pointed by ``ptr`` to this factory.)""";
        } Add;
        // Symbol: drake::symbolic::ExpressionAddFactory::AddExpression
        struct /* AddExpression */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Adds ``e`` to this factory.)""";
        } AddExpression;
        // Symbol: drake::symbolic::ExpressionAddFactory::ExpressionAddFactory
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_0args = R"""(Default constructor.)""";
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_2args =
R"""(Constructs ExpressionAddFactory with ``constant`` and
``expr_to_coeff_map``.)""";
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_1args =
R"""(Constructs ExpressionAddFactory from ``add``.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAddFactory::GetExpression
        struct /* GetExpression */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns a symbolic expression.)""";
        } GetExpression;
        // Symbol: drake::symbolic::ExpressionAddFactory::Negate
        struct /* Negate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Negates the expressions in factory. If it represents c0 + c1 * t1 +
... + * cn * tn, this method flips it into -c0 - c1 * t1 - ... - cn *
tn.

Returns:
    *this.)""";
        } Negate;
      } ExpressionAddFactory;
      // Symbol: drake::symbolic::ExpressionAsin
      struct /* ExpressionAsin */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing arcsine function.)""";
        // Symbol: drake::symbolic::ExpressionAsin::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionAsin::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionAsin::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionAsin::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionAsin::ExpressionAsin
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAsin::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionAsin;
      // Symbol: drake::symbolic::ExpressionAtan
      struct /* ExpressionAtan */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing arctangent function.)""";
        // Symbol: drake::symbolic::ExpressionAtan::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionAtan::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionAtan::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionAtan::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionAtan::ExpressionAtan
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAtan::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionAtan;
      // Symbol: drake::symbolic::ExpressionAtan2
      struct /* ExpressionAtan2 */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing atan2 function (arctangent function
with two arguments). atan2(y, x) is defined as atan(y/x).)""";
        // Symbol: drake::symbolic::ExpressionAtan2::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionAtan2::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionAtan2::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionAtan2::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionAtan2::ExpressionAtan2
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionAtan2::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionAtan2;
      // Symbol: drake::symbolic::ExpressionCeiling
      struct /* ExpressionCeiling */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing ceil function.)""";
        // Symbol: drake::symbolic::ExpressionCeiling::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionCeiling::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionCeiling::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionCeiling::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionCeiling::ExpressionCeiling
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionCeiling::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionCeiling;
      // Symbol: drake::symbolic::ExpressionCell
      struct /* ExpressionCell */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Represents an abstract class which is the base of concrete
symbolic-expression classes.

Note:
    It provides virtual function, ExpressionCell∷Display, because
    operator<< is not allowed to be a virtual function.)""";
        // Symbol: drake::symbolic::ExpressionCell::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Differentiates this symbolic expression with respect to the variable
``var``.

Raises:
    RuntimeError if it is not differentiable.)""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionCell::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Outputs string representation of expression into output stream ``os``.)""";
        } Display;
        // Symbol: drake::symbolic::ExpressionCell::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Checks structural equality.)""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionCell::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Evaluates under a given environment (by default, an empty
environment).

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionCell::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns an Expression obtained by replacing all occurrences of the
variables in ``env`` in the current expression cell with the
corresponding values in ``env``.

Raises:
    RuntimeError if NaN is detected during substitution.)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionCell::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Expands out products and positive integer powers in expression.

Raises:
    RuntimeError if NaN is detected during expansion.)""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionCell::ExpressionCell
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs ExpressionCell of kind ``k`` with ``is_poly`` and
``is_expanded``, with a ``use_count`` of zero.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionCell::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Collects variables in expression.)""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionCell::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Sends all hash-relevant bytes for this ExpressionCell type into the
given hasher, per the hash_append concept -- except for get_kind(),
because Expression already sends that.)""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionCell::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Provides lexicographical ordering between expressions.)""";
        } Less;
        // Symbol: drake::symbolic::ExpressionCell::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns an Expression obtained by replacing all occurrences of the
variables in ``s`` in the current expression cell with the
corresponding expressions in ``s``.

Raises:
    RuntimeError if NaN is detected during substitution.)""";
        } Substitute;
        // Symbol: drake::symbolic::ExpressionCell::get_kind
        struct /* get_kind */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns expression kind.)""";
        } get_kind;
        // Symbol: drake::symbolic::ExpressionCell::is_expanded
        struct /* is_expanded */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Checks if this symbolic expression is already expanded.)""";
        } is_expanded;
        // Symbol: drake::symbolic::ExpressionCell::is_polynomial
        struct /* is_polynomial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Checks if this symbolic expression is convertible to Polynomial.)""";
        } is_polynomial;
        // Symbol: drake::symbolic::ExpressionCell::set_expanded
        struct /* set_expanded */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Sets this symbolic expression as already expanded.)""";
        } set_expanded;
        // Symbol: drake::symbolic::ExpressionCell::use_count
        struct /* use_count */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns the intrusive use count (ala boost∷intrusive_ptr).)""";
        } use_count;
      } ExpressionCell;
      // Symbol: drake::symbolic::ExpressionCos
      struct /* ExpressionCos */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing cosine function.)""";
        // Symbol: drake::symbolic::ExpressionCos::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionCos::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionCos::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionCos::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionCos::ExpressionCos
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionCos::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionCos;
      // Symbol: drake::symbolic::ExpressionCosh
      struct /* ExpressionCosh */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing hyperbolic cosine function.)""";
        // Symbol: drake::symbolic::ExpressionCosh::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionCosh::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionCosh::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionCosh::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionCosh::ExpressionCosh
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionCosh::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionCosh;
      // Symbol: drake::symbolic::ExpressionDiv
      struct /* ExpressionDiv */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing division.)""";
        // Symbol: drake::symbolic::ExpressionDiv::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionDiv::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionDiv::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionDiv::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionDiv::ExpressionDiv
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionDiv::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionDiv;
      // Symbol: drake::symbolic::ExpressionExp
      struct /* ExpressionExp */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing exponentiation using the base of
natural logarithms.)""";
        // Symbol: drake::symbolic::ExpressionExp::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionExp::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionExp::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionExp::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionExp::ExpressionExp
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionExp::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionExp;
      // Symbol: drake::symbolic::ExpressionFloor
      struct /* ExpressionFloor */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing floor function.)""";
        // Symbol: drake::symbolic::ExpressionFloor::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionFloor::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionFloor::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionFloor::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionFloor::ExpressionFloor
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionFloor::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionFloor;
      // Symbol: drake::symbolic::ExpressionIfThenElse
      struct /* ExpressionIfThenElse */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing if-then-else expression.)""";
        // Symbol: drake::symbolic::ExpressionIfThenElse::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionIfThenElse::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionIfThenElse::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionIfThenElse::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionIfThenElse::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionIfThenElse::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionIfThenElse::ExpressionIfThenElse
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs if-then-else expression from ``f_cond``, ``e_then``, and
``e_else``.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionIfThenElse::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionIfThenElse::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionIfThenElse::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::ExpressionIfThenElse::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::ExpressionIfThenElse::get_conditional_formula
        struct /* get_conditional_formula */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the conditional formula.)""";
        } get_conditional_formula;
        // Symbol: drake::symbolic::ExpressionIfThenElse::get_else_expression
        struct /* get_else_expression */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the 'else' expression.)""";
        } get_else_expression;
        // Symbol: drake::symbolic::ExpressionIfThenElse::get_then_expression
        struct /* get_then_expression */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the 'then' expression.)""";
        } get_then_expression;
      } ExpressionIfThenElse;
      // Symbol: drake::symbolic::ExpressionKind
      struct /* ExpressionKind */ {
        // Source: drake/common/symbolic/expression/expression_kind.h
        const char* doc =
R"""(Kinds of symbolic expressions. The constants here are carefully chosen
to support nanboxing. For all elements except Constant, the bit
pattern must have have 0x7FF0 bits set but must not be exactly 0x7FF0
nor 0xFFF0 (reserved for ±infinity). Refer to the details in
boxed_cell.h for more information.)""";
        // Symbol: drake::symbolic::ExpressionKind::Abs
        struct /* Abs */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(absolute value function)""";
        } Abs;
        // Symbol: drake::symbolic::ExpressionKind::Acos
        struct /* Acos */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(arccosine)""";
        } Acos;
        // Symbol: drake::symbolic::ExpressionKind::Add
        struct /* Add */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(addition (+))""";
        } Add;
        // Symbol: drake::symbolic::ExpressionKind::Asin
        struct /* Asin */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(arcsine)""";
        } Asin;
        // Symbol: drake::symbolic::ExpressionKind::Atan
        struct /* Atan */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(arctangent)""";
        } Atan;
        // Symbol: drake::symbolic::ExpressionKind::Atan2
        struct /* Atan2 */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(arctangent2 (atan2(y,x) = atan(y/x)))""";
        } Atan2;
        // Symbol: drake::symbolic::ExpressionKind::Ceil
        struct /* Ceil */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(ceil)""";
        } Ceil;
        // Symbol: drake::symbolic::ExpressionKind::Constant
        struct /* Constant */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(constant (double))""";
        } Constant;
        // Symbol: drake::symbolic::ExpressionKind::Cos
        struct /* Cos */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(cosine)""";
        } Cos;
        // Symbol: drake::symbolic::ExpressionKind::Cosh
        struct /* Cosh */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(hyperbolic cosine)""";
        } Cosh;
        // Symbol: drake::symbolic::ExpressionKind::Div
        struct /* Div */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(division (/))""";
        } Div;
        // Symbol: drake::symbolic::ExpressionKind::Exp
        struct /* Exp */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(exponentiation)""";
        } Exp;
        // Symbol: drake::symbolic::ExpressionKind::Floor
        struct /* Floor */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(floor)""";
        } Floor;
        // Symbol: drake::symbolic::ExpressionKind::IfThenElse
        struct /* IfThenElse */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(if then else)""";
        } IfThenElse;
        // Symbol: drake::symbolic::ExpressionKind::Log
        struct /* Log */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(logarithms)""";
        } Log;
        // Symbol: drake::symbolic::ExpressionKind::Max
        struct /* Max */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(max)""";
        } Max;
        // Symbol: drake::symbolic::ExpressionKind::Min
        struct /* Min */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(min)""";
        } Min;
        // Symbol: drake::symbolic::ExpressionKind::Mul
        struct /* Mul */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(multiplication (*))""";
        } Mul;
        // Symbol: drake::symbolic::ExpressionKind::NaN
        struct /* NaN */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(NaN)""";
        } NaN;
        // Symbol: drake::symbolic::ExpressionKind::Pow
        struct /* Pow */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(power function)""";
        } Pow;
        // Symbol: drake::symbolic::ExpressionKind::Sin
        struct /* Sin */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(sine)""";
        } Sin;
        // Symbol: drake::symbolic::ExpressionKind::Sinh
        struct /* Sinh */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(hyperbolic sine)""";
        } Sinh;
        // Symbol: drake::symbolic::ExpressionKind::Sqrt
        struct /* Sqrt */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(square root)""";
        } Sqrt;
        // Symbol: drake::symbolic::ExpressionKind::Tan
        struct /* Tan */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(tangent)""";
        } Tan;
        // Symbol: drake::symbolic::ExpressionKind::Tanh
        struct /* Tanh */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(hyperbolic tangent)""";
        } Tanh;
        // Symbol: drake::symbolic::ExpressionKind::UninterpretedFunction
        struct /* UninterpretedFunction */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(Uninterpreted function)""";
        } UninterpretedFunction;
        // Symbol: drake::symbolic::ExpressionKind::Var
        struct /* Var */ {
          // Source: drake/common/symbolic/expression/expression_kind.h
          const char* doc = R"""(variable)""";
        } Var;
      } ExpressionKind;
      // Symbol: drake::symbolic::ExpressionLog
      struct /* ExpressionLog */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing logarithms.)""";
        // Symbol: drake::symbolic::ExpressionLog::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionLog::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionLog::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionLog::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionLog::ExpressionLog
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionLog::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionLog;
      // Symbol: drake::symbolic::ExpressionMax
      struct /* ExpressionMax */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing max function.)""";
        // Symbol: drake::symbolic::ExpressionMax::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionMax::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionMax::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionMax::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionMax::ExpressionMax
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionMax::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionMax;
      // Symbol: drake::symbolic::ExpressionMin
      struct /* ExpressionMin */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing min function.)""";
        // Symbol: drake::symbolic::ExpressionMin::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionMin::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionMin::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionMin::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionMin::ExpressionMin
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionMin::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionMin;
      // Symbol: drake::symbolic::ExpressionMul
      struct /* ExpressionMul */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing a multiplication of powers.

.. math:: c_0 \cdot \prod b_i^{e_i}

where :math:`c_0` is a constant and :math:`b_i` and :math:`e_i` are
symbolic expressions.

Internally this class maintains a member variable ``constant_``
representing :math:`c_0` and another member variable
``base_to_exponent_map_`` representing a mapping from a base,
:math:`b_i` to its exponentiation :math:`e_i`.)""";
        // Symbol: drake::symbolic::ExpressionMul::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionMul::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionMul::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionMul::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionMul::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionMul::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionMul::ExpressionMul
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs ExpressionMul from ``constant`` and
``base_to_exponent_map``.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionMul::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionMul::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionMul::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::ExpressionMul::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::ExpressionMul::get_base_to_exponent_map
        struct /* get_base_to_exponent_map */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns map from a term to its coefficient.)""";
        } get_base_to_exponent_map;
        // Symbol: drake::symbolic::ExpressionMul::get_constant
        struct /* get_constant */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns constant term.)""";
        } get_constant;
      } ExpressionMul;
      // Symbol: drake::symbolic::ExpressionMulFactory
      struct /* ExpressionMulFactory */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Factory class to help build ExpressionMul expressions.)""";
        // Symbol: drake::symbolic::ExpressionMulFactory::Add
        struct /* Add */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Adds ExpressionMul pointed by ``ptr`` to this factory.)""";
        } Add;
        // Symbol: drake::symbolic::ExpressionMulFactory::AddExpression
        struct /* AddExpression */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Adds ``e`` to this factory.)""";
        } AddExpression;
        // Symbol: drake::symbolic::ExpressionMulFactory::ExpressionMulFactory
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_0args = R"""(Default constructor.)""";
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_2args_constant_base_to_exponent_map =
R"""(Constructs ExpressionMulFactory with ``constant`` and
Expression-valued ``base_to_exponent_map``. Note that this constructor
runs in constant-time because it moves the map into storage; it does
not loop over the map.)""";
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_1args_base_to_exponent_map =
R"""(Constructs ExpressionMulFactory with a Monomial-like (Variable to
integer power) ``base_to_exponent_map``.)""";
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc_1args_mul =
R"""(Constructs ExpressionMulFactory from ``mul``.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionMulFactory::GetExpression
        struct /* GetExpression */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns a symbolic expression.)""";
        } GetExpression;
        // Symbol: drake::symbolic::ExpressionMulFactory::Negate
        struct /* Negate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Negates the expressions in factory. If it represents c0 * p1 * ... *
pn, this method flips it into -c0 * p1 * ... * pn.

Returns:
    *this.)""";
        } Negate;
      } ExpressionMulFactory;
      // Symbol: drake::symbolic::ExpressionNaN
      struct /* ExpressionNaN */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing NaN (not-a-number).)""";
        // Symbol: drake::symbolic::ExpressionNaN::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionNaN::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionNaN::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionNaN::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionNaN::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionNaN::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionNaN::ExpressionNaN
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionNaN::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionNaN::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionNaN::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::ExpressionNaN::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionNaN;
      // Symbol: drake::symbolic::ExpressionPow
      struct /* ExpressionPow */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing power function.)""";
        // Symbol: drake::symbolic::ExpressionPow::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionPow::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionPow::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionPow::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionPow::ExpressionPow
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionPow::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionPow;
      // Symbol: drake::symbolic::ExpressionSin
      struct /* ExpressionSin */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing sine function.)""";
        // Symbol: drake::symbolic::ExpressionSin::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionSin::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionSin::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionSin::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionSin::ExpressionSin
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionSin::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionSin;
      // Symbol: drake::symbolic::ExpressionSinh
      struct /* ExpressionSinh */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing hyperbolic sine function.)""";
        // Symbol: drake::symbolic::ExpressionSinh::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionSinh::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionSinh::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionSinh::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionSinh::ExpressionSinh
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionSinh::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionSinh;
      // Symbol: drake::symbolic::ExpressionSqrt
      struct /* ExpressionSqrt */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing square-root.)""";
        // Symbol: drake::symbolic::ExpressionSqrt::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionSqrt::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionSqrt::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionSqrt::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionSqrt::ExpressionSqrt
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionSqrt::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionSqrt;
      // Symbol: drake::symbolic::ExpressionTan
      struct /* ExpressionTan */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing tangent function.)""";
        // Symbol: drake::symbolic::ExpressionTan::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionTan::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionTan::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionTan::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionTan::ExpressionTan
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionTan::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionTan;
      // Symbol: drake::symbolic::ExpressionTanh
      struct /* ExpressionTanh */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing hyperbolic tangent function.)""";
        // Symbol: drake::symbolic::ExpressionTanh::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionTanh::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionTanh::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionTanh::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionTanh::ExpressionTanh
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionTanh::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } ExpressionTanh;
      // Symbol: drake::symbolic::ExpressionUninterpretedFunction
      struct /* ExpressionUninterpretedFunction */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing an uninterpreted function.)""";
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::ExpressionUninterpretedFunction
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs an uninterpreted-function expression from ``name`` and
``arguments``.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::get_arguments
        struct /* get_arguments */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns the arguments of this expression.)""";
        } get_arguments;
        // Symbol: drake::symbolic::ExpressionUninterpretedFunction::get_name
        struct /* get_name */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the name of this expression.)""";
        } get_name;
      } ExpressionUninterpretedFunction;
      // Symbol: drake::symbolic::ExpressionVar
      struct /* ExpressionVar */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Symbolic expression representing a variable.)""";
        // Symbol: drake::symbolic::ExpressionVar::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Differentiate;
        // Symbol: drake::symbolic::ExpressionVar::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::ExpressionVar::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::ExpressionVar::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::ExpressionVar::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ExpressionVar::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Expand;
        // Symbol: drake::symbolic::ExpressionVar::ExpressionVar
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs an expression from ``var``.

Precondition:
    ``var`` is not a BOOLEAN variable.)""";
        } ctor;
        // Symbol: drake::symbolic::ExpressionVar::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::ExpressionVar::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::ExpressionVar::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::ExpressionVar::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::ExpressionVar::get_variable
        struct /* get_variable */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } get_variable;
      } ExpressionVar;
      // Symbol: drake::symbolic::Formula
      struct /* Formula */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Represents a symbolic form of a first-order logic formula.

It has the following grammar:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    F := ⊥ | ⊤ | Var | E = E | E ≠ E | E > E | E ≥ E | E < E | E ≤ E
    | E ∧ ... ∧ E | E ∨ ... ∨ E | ¬F | ∀ x₁, ..., xn. F

.. raw:: html

    </details>

In the implementation, Formula is a simple wrapper including a shared
pointer to FormulaCell class which is a super-class of different kinds
of symbolic formulas (i.e. FormulaAnd, FormulaOr, FormulaEq). Note
that it includes a shared pointer, not a unique pointer, to allow
sharing sub-expressions.

Note:
    The sharing of sub-expressions is not yet implemented.

The following simple simplifications are implemented:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    E1 = E2        ->  True    (if E1 and E2 are structurally equal)
    E1 ≠ E2        ->  False   (if E1 and E2 are structurally equal)
    E1 > E2        ->  False   (if E1 and E2 are structurally equal)
    E1 ≥ E2        ->  True    (if E1 and E2 are structurally equal)
    E1 < E2        ->  False   (if E1 and E2 are structurally equal)
    E1 ≤ E2        ->  True    (if E1 and E2 are structurally equal)
    F1 ∧ F2        ->  False   (if either F1 or F2 is False)
    F1 ∨ F2        ->  True    (if either F1 or F2 is True)
    ¬(¬(F))        ->  F

.. raw:: html

    </details>

We flatten nested conjunctions (or disjunctions) at the construction.
A conjunction (resp. disjunction) takes a set of conjuncts (resp.
disjuncts). Note that any duplicated conjunct/disjunct is removed. For
example, both of ``f1 && (f2 && f1)`` and ``(f1 && f2) && f1`` are
flattened to ``f1 && f2 && f1`` and simplified into ``f1 && f2``. As a
result, the two are identified as the same formula.

Note:
    Formula class has an explicit conversion operator to bool. It
    evaluates a symbolic formula under an empty environment. If a
    symbolic formula includes variables, the conversion operator
    throws an exception. This operator is only intended for
    third-party code doing things like ``(imag(SymbolicExpression(0))
    == SymbolicExpression(0)) { ... };`` that we found in Eigen3
    codebase. In general, a user of this class should explicitly call
    ``Evaluate`` from within Drake for readability.)""";
        // Symbol: drake::symbolic::Formula::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Checks structural equality.)""";
        } EqualTo;
        // Symbol: drake::symbolic::Formula::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_2args =
R"""(Evaluates using a given environment (by default, an empty environment)
and a random number generator. If there is a random variable in this
formula which is unassigned in ``env``, it uses ``random_generator``
to sample a value and use it to substitute all occurrences of the
random variable in this formula.

Raises:
    RuntimeError if a variable ``v`` is needed for an evaluation but
    not provided by ``env``.

Raises:
    RuntimeError if an unassigned random variable is detected while
    ``random_generator`` is ``nullptr``.)""";
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_1args =
R"""(Evaluates using an empty environment and a random number generator.

See the above overload for the exceptions that it might throw.)""";
        } Evaluate;
        // Symbol: drake::symbolic::Formula::False
        struct /* False */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""()""";
        } False;
        // Symbol: drake::symbolic::Formula::Formula
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_0args =
R"""(Default constructor. Sets the value to Formula∷False, to be consistent
with value-initialized `bool`s.)""";
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_1args_value =
R"""(Constructs from a ``bool``. This overload is also used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_1args_var =
R"""(Constructs a formula from ``var``.

Precondition:
    ``var`` is of BOOLEAN type.)""";
        } ctor;
        // Symbol: drake::symbolic::Formula::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc =
R"""(Gets free variables (unquantified variables).)""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::Formula::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc =
R"""(Checks lexicographical ordering between this and ``e``.

If the two formulas f1 and f2 have different kinds k1 and k2
respectively, f1.Less(f2) is equal to k1 < k2. If f1 and f2 are
expressions of the same kind, we check the ordering between f1 and f2
by comparing their elements lexicographically.

For example, in case of And, let f1 and f2 be

f1 = f_1,1 ∧ ... ∧ f_1,n f2 = f_2,1 ∧ ... ∧ f_2,m

f1.Less(f2) is true if there exists an index i (<= n, m) such that for
all j < i, we have

¬(f_1_j.Less(f_2_j)) ∧ ¬(f_2_j.Less(f_1_j))

and f_1_i.Less(f_2_i) holds.

This function is used as a compare function in
std∷map<symbolic∷Formula> and std∷set<symbolic∷Formula> via
std∷less<symbolic∷Formula>.)""";
        } Less;
        // Symbol: drake::symbolic::Formula::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_2args =
R"""(Returns a copy of this formula replacing all occurrences of ``var``
with ``e``.

Raises:
    RuntimeError if NaN is detected during substitution.)""";
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc_1args =
R"""(Returns a copy of this formula replacing all occurrences of the
variables in ``s`` with corresponding expressions in ``s``. Note that
the substitutions occur simultaneously. For example, (x / y >
0).Substitute({{x, y}, {y, x}}) gets (y / x > 0).

Raises:
    RuntimeError if NaN is detected during substitution.)""";
        } Substitute;
        // Symbol: drake::symbolic::Formula::True
        struct /* True */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""()""";
        } True;
        // Symbol: drake::symbolic::Formula::get_kind
        struct /* get_kind */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""()""";
        } get_kind;
        // Symbol: drake::symbolic::Formula::operator bool
        struct /* operator_bool */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Conversion to bool.)""";
        } operator_bool;
        // Symbol: drake::symbolic::Formula::to_string
        struct /* to_string */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc =
R"""(Returns string representation of Formula.)""";
        } to_string;
      } Formula;
      // Symbol: drake::symbolic::FormulaAnd
      struct /* FormulaAnd */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing conjunctions (f1 ∧ ... ∧ fn).)""";
        // Symbol: drake::symbolic::FormulaAnd::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaAnd::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaAnd::FormulaAnd
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_1args = R"""(Constructs from ``formulas``.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_2args = R"""(Constructs ``f1`` ∧ ``f2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaAnd::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaAnd;
      // Symbol: drake::symbolic::FormulaCell
      struct /* FormulaCell */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Represents an abstract class which is the base of concrete
symbolic-formula classes (i.e. symbolic∷FormulaAnd,
symbolic∷FormulaEq).

Note:
    It provides virtual function, FormulaCell∷Display, because
    operator<< is not allowed to be a virtual function.)""";
        // Symbol: drake::symbolic::FormulaCell::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Outputs string representation of formula into output stream ``os``.)""";
        } Display;
        // Symbol: drake::symbolic::FormulaCell::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Checks structural equality.)""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaCell::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Evaluates under a given environment.)""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaCell::FormulaCell
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_0args = R"""(Default constructor (deleted).)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_move =
R"""(Move-construct a formula from an rvalue.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_copy =
R"""(Copy-construct a formula from an lvalue.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_1args = R"""(Construct FormulaCell of kind ``k``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaCell::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Returns set of free variables in formula.)""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaCell::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Sends all hash-relevant bytes for this FormulaCell type into the given
hasher, per the hash_append concept -- except for get_kind(), because
Formula already sends that.)""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaCell::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Checks ordering.)""";
        } Less;
        // Symbol: drake::symbolic::FormulaCell::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Returns a Formula obtained by replacing all occurrences of the
variables in ``s`` in the current formula cell with the corresponding
expressions in ``s``.)""";
        } Substitute;
        // Symbol: drake::symbolic::FormulaCell::get_kind
        struct /* get_kind */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Returns kind of formula.)""";
        } get_kind;
      } FormulaCell;
      // Symbol: drake::symbolic::FormulaEq
      struct /* FormulaEq */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing equality (e1 = e2).)""";
        // Symbol: drake::symbolic::FormulaEq::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaEq::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaEq::FormulaEq
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``e1`` and ``e2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaEq::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaEq;
      // Symbol: drake::symbolic::FormulaFalse
      struct /* FormulaFalse */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc = R"""(Symbolic formula representing false.)""";
        // Symbol: drake::symbolic::FormulaFalse::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaFalse::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaFalse::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaFalse::FormulaFalse
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Default Constructor.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaFalse::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaFalse::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaFalse::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::FormulaFalse::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaFalse;
      // Symbol: drake::symbolic::FormulaForall
      struct /* FormulaForall */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing universal quantifications (∀ x₁, ..., *
xn. F).)""";
        // Symbol: drake::symbolic::FormulaForall::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaForall::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaForall::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaForall::FormulaForall
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``vars`` and ``f``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaForall::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaForall::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaForall::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::FormulaForall::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::FormulaForall::get_quantified_formula
        struct /* get_quantified_formula */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Returns the quantified formula.)""";
        } get_quantified_formula;
        // Symbol: drake::symbolic::FormulaForall::get_quantified_variables
        struct /* get_quantified_variables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Returns the quantified variables.)""";
        } get_quantified_variables;
      } FormulaForall;
      // Symbol: drake::symbolic::FormulaGeq
      struct /* FormulaGeq */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing 'greater-than-or-equal-to' (e1 ≥ e2).)""";
        // Symbol: drake::symbolic::FormulaGeq::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaGeq::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaGeq::FormulaGeq
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``e1`` and ``e2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaGeq::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaGeq;
      // Symbol: drake::symbolic::FormulaGt
      struct /* FormulaGt */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing 'greater-than' (e1 > e2).)""";
        // Symbol: drake::symbolic::FormulaGt::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaGt::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaGt::FormulaGt
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``e1`` and ``e2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaGt::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaGt;
      // Symbol: drake::symbolic::FormulaIsnan
      struct /* FormulaIsnan */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing isnan predicate.)""";
        // Symbol: drake::symbolic::FormulaIsnan::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaIsnan::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaIsnan::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaIsnan::FormulaIsnan
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::FormulaIsnan::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaIsnan::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaIsnan::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::FormulaIsnan::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::FormulaIsnan::get_unary_expression
        struct /* get_unary_expression */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Returns the operand expression.)""";
        } get_unary_expression;
      } FormulaIsnan;
      // Symbol: drake::symbolic::FormulaKind
      struct /* FormulaKind */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""(Kinds of symbolic formulas.)""";
        // Symbol: drake::symbolic::FormulaKind::And
        struct /* And */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Conjunction (∧))""";
        } And;
        // Symbol: drake::symbolic::FormulaKind::Eq
        struct /* Eq */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(=)""";
        } Eq;
        // Symbol: drake::symbolic::FormulaKind::False
        struct /* False */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(⊥)""";
        } False;
        // Symbol: drake::symbolic::FormulaKind::Forall
        struct /* Forall */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Universal quantification (∀))""";
        } Forall;
        // Symbol: drake::symbolic::FormulaKind::Geq
        struct /* Geq */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(>=)""";
        } Geq;
        // Symbol: drake::symbolic::FormulaKind::Gt
        struct /* Gt */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(>)""";
        } Gt;
        // Symbol: drake::symbolic::FormulaKind::Isnan
        struct /* Isnan */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(NaN check predicate)""";
        } Isnan;
        // Symbol: drake::symbolic::FormulaKind::Leq
        struct /* Leq */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(<=)""";
        } Leq;
        // Symbol: drake::symbolic::FormulaKind::Lt
        struct /* Lt */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(<)""";
        } Lt;
        // Symbol: drake::symbolic::FormulaKind::Neq
        struct /* Neq */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(!=)""";
        } Neq;
        // Symbol: drake::symbolic::FormulaKind::Not
        struct /* Not */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Negation (¬))""";
        } Not;
        // Symbol: drake::symbolic::FormulaKind::Or
        struct /* Or */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Disjunction (∨))""";
        } Or;
        // Symbol: drake::symbolic::FormulaKind::PositiveSemidefinite
        struct /* PositiveSemidefinite */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Positive semidefinite matrix)""";
        } PositiveSemidefinite;
        // Symbol: drake::symbolic::FormulaKind::True
        struct /* True */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(⊤)""";
        } True;
        // Symbol: drake::symbolic::FormulaKind::Var
        struct /* Var */ {
          // Source: drake/common/symbolic/expression/formula.h
          const char* doc = R"""(Boolean Variable)""";
        } Var;
      } FormulaKind;
      // Symbol: drake::symbolic::FormulaLeq
      struct /* FormulaLeq */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing 'less-than-or-equal-to' (e1 ≤ e2).)""";
        // Symbol: drake::symbolic::FormulaLeq::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaLeq::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaLeq::FormulaLeq
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``e1`` and ``e2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaLeq::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaLeq;
      // Symbol: drake::symbolic::FormulaLt
      struct /* FormulaLt */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing 'less-than' (e1 < e2).)""";
        // Symbol: drake::symbolic::FormulaLt::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaLt::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaLt::FormulaLt
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``e1`` and ``e2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaLt::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaLt;
      // Symbol: drake::symbolic::FormulaNeq
      struct /* FormulaNeq */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing disequality (e1 ≠ e2).)""";
        // Symbol: drake::symbolic::FormulaNeq::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaNeq::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaNeq::FormulaNeq
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``e1`` and ``e2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaNeq::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaNeq;
      // Symbol: drake::symbolic::FormulaNot
      struct /* FormulaNot */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing negations (¬f).)""";
        // Symbol: drake::symbolic::FormulaNot::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaNot::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaNot::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaNot::FormulaNot
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Constructs from ``f``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaNot::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaNot::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaNot::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::FormulaNot::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::FormulaNot::get_operand
        struct /* get_operand */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Returns the operand.)""";
        } get_operand;
      } FormulaNot;
      // Symbol: drake::symbolic::FormulaOr
      struct /* FormulaOr */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing disjunctions (f1 ∨ ... ∨ fn).)""";
        // Symbol: drake::symbolic::FormulaOr::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaOr::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaOr::FormulaOr
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_1args = R"""(Constructs from ``formulas``.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_2args = R"""(Constructs ``f1`` ∨ ``f2``.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaOr::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaOr;
      // Symbol: drake::symbolic::FormulaPositiveSemidefinite
      struct /* FormulaPositiveSemidefinite */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing positive-semidefinite (PSD) constraint.)""";
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::FormulaPositiveSemidefinite
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Constructs a positive-semidefinite formula from a symmetric matrix
``m``.

Raises:
    RuntimeError if ``m`` is not symmetric.

Note:
    This constructor checks if ``m`` is symmetric, which can be
    costly.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Checks ordering between this PSD formula and ``f``. The ordering
between two PSD formulas ``psd1`` and ``psd2`` are determined by the
ordering between the two matrices ``m1`` in ``psd1`` and ``m2`` in
``psd2``.

First, we compare the size of ``m1`` and ``m2``:

- If ``m1`` is smaller than ``m2``, `psd1.Less(psd2)` is true.
- If ``m2`` is smaller than ``m1``, `psd1.Less(psd2)` is false.

If ``m1`` and ``m2`` are of the same size, we perform element-wise
comparison by following column-major order. See the following example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    m1 << x + y, -3.14,
          -3.14,     y;
    m2 << x + y,  3.14,
           3.14,     y;
    const Formula psd1{positive_semidefinite(m1)};
    const Formula psd2{positive_semidefinite(m2)};
    
    EXPECT_TRUE(psd1.Less(psd2));

.. raw:: html

    </details>

Note:
    In the code above, ``psd1.Less(psd2)`` holds because we have

- m1 in column-major ordering : (x + y) -3.14 -3.14 y_ - m2 in
column-major ordering : (x + y) 3.14 3.14 y_.)""";
        } Less;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::FormulaPositiveSemidefinite::get_matrix
        struct /* get_matrix */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Returns the corresponding matrix in this PSD formula.)""";
        } get_matrix;
      } FormulaPositiveSemidefinite;
      // Symbol: drake::symbolic::FormulaTrue
      struct /* FormulaTrue */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc = R"""(Symbolic formula representing true.)""";
        // Symbol: drake::symbolic::FormulaTrue::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaTrue::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaTrue::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaTrue::FormulaTrue
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Default Constructor.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaTrue::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaTrue::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaTrue::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::FormulaTrue::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
      } FormulaTrue;
      // Symbol: drake::symbolic::FormulaVar
      struct /* FormulaVar */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Symbolic formula representing a Boolean variable.)""";
        // Symbol: drake::symbolic::FormulaVar::Display
        struct /* Display */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Display;
        // Symbol: drake::symbolic::FormulaVar::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::FormulaVar::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::FormulaVar::FormulaVar
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Constructs a formula from ``var``.

Precondition:
    ``var`` is of BOOLEAN type.)""";
        } ctor;
        // Symbol: drake::symbolic::FormulaVar::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::FormulaVar::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::FormulaVar::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::FormulaVar::Substitute
        struct /* Substitute */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Substitute;
        // Symbol: drake::symbolic::FormulaVar::get_variable
        struct /* get_variable */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } get_variable;
      } FormulaVar;
      // Symbol: drake::symbolic::GetDistinctVariables
      struct /* GetDistinctVariables */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the distinct variables in the matrix of expressions.)""";
      } GetDistinctVariables;
      // Symbol: drake::symbolic::GetVariableVector
      struct /* GetVariableVector */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Constructs a vector of variables from the vector of variable
expressions.

Raises:
    RuntimeError if there is an expression in ``vec`` which is not a
    variable.)""";
      } GetVariableVector;
      // Symbol: drake::symbolic::Jacobian
      struct /* Jacobian */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Computes the Jacobian matrix J of the vector function ``f`` with
respect to ``vars``. J(i,j) contains ∂f(i)/∂vars(j).

For example, Jacobian([x * cos(y), x * sin(y), x^2], {x, y}) returns
the following 3x2 matrix:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    = |cos(y)   -x * sin(y)|
       |sin(y)    x * cos(y)|
       | 2 * x             0|

.. raw:: html

    </details>

Precondition:
    {``vars`` is non-empty}.)""";
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_expression =
R"""(Computes the Jacobian matrix J of the vector function ``f`` with
respect to ``vars``. J(i,j) contains ∂f(i)/∂vars(j).

Precondition:
    {``vars`` is non-empty}.)""";
      } Jacobian;
      // Symbol: drake::symbolic::MakeMatrixBinaryVariable
      struct /* MakeMatrixBinaryVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_3args =
R"""(Creates a dynamically-sized Eigen matrix of symbolic binary variables.

Parameter ``rows``:
    The number of rows in the new matrix.

Parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen matrix of symbolic binary variables.

Template parameter ``rows``:
    The number of rows in the new matrix.

Template parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
      } MakeMatrixBinaryVariable;
      // Symbol: drake::symbolic::MakeMatrixBooleanVariable
      struct /* MakeMatrixBooleanVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_3args =
R"""(Creates a dynamically-sized Eigen matrix of symbolic Boolean
variables.

Parameter ``rows``:
    The number of rows in the new matrix.

Parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen matrix of symbolic Boolean variables.

Template parameter ``rows``:
    The number of rows in the new matrix.

Template parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
      } MakeMatrixBooleanVariable;
      // Symbol: drake::symbolic::MakeMatrixContinuousVariable
      struct /* MakeMatrixContinuousVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_3args =
R"""(Creates a dynamically-sized Eigen matrix of symbolic continuous
variables.

Parameter ``rows``:
    The number of rows in the new matrix.

Parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen matrix of symbolic continuous variables.

Template parameter ``rows``:
    The number of rows in the new matrix.

Template parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
      } MakeMatrixContinuousVariable;
      // Symbol: drake::symbolic::MakeMatrixIntegerVariable
      struct /* MakeMatrixIntegerVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_3args =
R"""(Creates a dynamically-sized Eigen matrix of symbolic integer
variables.

Parameter ``rows``:
    The number of rows in the new matrix.

Parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen matrix of symbolic integer variables.

Template parameter ``rows``:
    The number of rows in the new matrix.

Template parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.)""";
      } MakeMatrixIntegerVariable;
      // Symbol: drake::symbolic::MakeMatrixVariable
      struct /* MakeMatrixVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_4args =
R"""(Creates a dynamically-sized Eigen matrix of symbolic variables.

Parameter ``rows``:
    The number of rows in the new matrix.

Parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.

Parameter ``type``:
    The type of variables in the matrix.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_2args =
R"""(Creates a static-sized Eigen matrix of symbolic variables.

Template parameter ``rows``:
    The number of rows in the new matrix.

Template parameter ``cols``:
    The number of cols in the new matrix.

Parameter ``name``:
    The common prefix for variables. The (i, j)-th element will be
    named as ``name(i, j)``.

Parameter ``type``:
    The type of variables in the matrix.)""";
      } MakeMatrixVariable;
      // Symbol: drake::symbolic::MakeVectorBinaryVariable
      struct /* MakeVectorBinaryVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_2args =
R"""(Creates a dynamically-sized Eigen vector of symbolic binary variables.

Parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen vector of symbolic binary variables.

Template parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
      } MakeVectorBinaryVariable;
      // Symbol: drake::symbolic::MakeVectorBooleanVariable
      struct /* MakeVectorBooleanVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_2args =
R"""(Creates a dynamically-sized Eigen vector of symbolic Boolean
variables.

Parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen vector of symbolic Boolean variables.

Template parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
      } MakeVectorBooleanVariable;
      // Symbol: drake::symbolic::MakeVectorContinuousVariable
      struct /* MakeVectorContinuousVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_2args =
R"""(Creates a dynamically-sized Eigen vector of symbolic continuous
variables.

Parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen vector of symbolic continuous variables.

Template parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
      } MakeVectorContinuousVariable;
      // Symbol: drake::symbolic::MakeVectorIntegerVariable
      struct /* MakeVectorIntegerVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_2args =
R"""(Creates a dynamically-sized Eigen vector of symbolic integer
variables.

Parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_1args =
R"""(Creates a static-sized Eigen vector of symbolic integer variables.

Template parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.)""";
      } MakeVectorIntegerVariable;
      // Symbol: drake::symbolic::MakeVectorVariable
      struct /* MakeVectorVariable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_3args =
R"""(Creates a dynamically-sized Eigen vector of symbolic variables.

Parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.

Parameter ``type``:
    The type of variables in the vector.)""";
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc_2args =
R"""(Creates a static-sized Eigen vector of symbolic variables.

Template parameter ``rows``:
    The size of vector.

Parameter ``name``:
    The common prefix for variables. The i-th element will be named as
    ``name(i)``.

Parameter ``type``:
    The type of variables in the vector.)""";
      } MakeVectorVariable;
      // Symbol: drake::symbolic::NaryFormulaCell
      struct /* NaryFormulaCell */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Represents the base class for N-ary logic operators (∧ and ∨).

Note:
    Internally this class maintains a set of symbolic formulas to
    avoid duplicated elements (i.e. f1 ∧ ... ∧ f1).)""";
        // Symbol: drake::symbolic::NaryFormulaCell::DisplayWithOp
        struct /* DisplayWithOp */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } DisplayWithOp;
        // Symbol: drake::symbolic::NaryFormulaCell::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::NaryFormulaCell::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::NaryFormulaCell::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::NaryFormulaCell::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::NaryFormulaCell::NaryFormulaCell
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_0args = R"""(Default constructor (deleted).)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_move =
R"""(Move-construct a formula from an rvalue.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_copy =
R"""(Copy-construct a formula from an lvalue.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_2args =
R"""(Construct NaryFormulaCell of kind ``k`` with ``formulas``.)""";
        } ctor;
        // Symbol: drake::symbolic::NaryFormulaCell::get_operands
        struct /* get_operands */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""(Returns the formulas.)""";
        } get_operands;
      } NaryFormulaCell;
      // Symbol: drake::symbolic::PopulateRandomVariables
      struct /* PopulateRandomVariables */ {
        // Source: drake/common/symbolic/expression/environment.h
        const char* doc =
R"""(Populates the environment ``env`` by sampling values for the
unassigned random variables in ``variables`` using
``random_generator``.)""";
      } PopulateRandomVariables;
      // Symbol: drake::symbolic::RelationalFormulaCell
      struct /* RelationalFormulaCell */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Represents the base class for relational operators (==, !=, <, <=, >,
>=).)""";
        // Symbol: drake::symbolic::RelationalFormulaCell::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::RelationalFormulaCell::GetFreeVariables
        struct /* GetFreeVariables */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } GetFreeVariables;
        // Symbol: drake::symbolic::RelationalFormulaCell::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::RelationalFormulaCell::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::RelationalFormulaCell::RelationalFormulaCell
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_0args = R"""(Default constructor (deleted).)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_move =
R"""(Move-construct a formula from an rvalue.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_copy =
R"""(Copy-construct a formula from an lvalue.)""";
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc_3args =
R"""(Construct RelationalFormulaCell of kind ``k`` with ``lhs`` and
``rhs``.)""";
        } ctor;
        // Symbol: drake::symbolic::RelationalFormulaCell::get_lhs_expression
        struct /* get_lhs_expression */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Returns the expression on left-hand-side.)""";
        } get_lhs_expression;
        // Symbol: drake::symbolic::RelationalFormulaCell::get_rhs_expression
        struct /* get_rhs_expression */ {
          // Source: drake/common/symbolic/expression/formula_cell.h
          const char* doc =
R"""(Returns the expression on right-hand-side.)""";
        } get_rhs_expression;
      } RelationalFormulaCell;
      // Symbol: drake::symbolic::Substitute
      struct /* Substitute */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_2args =
R"""(Substitutes a symbolic matrix ``m`` using a given substitution
``subst``.

Returns:
    a matrix of symbolic expressions whose size is the size of ``m``.

Raises:
    RuntimeError if NaN is detected during substitution.)""";
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_3args =
R"""(Substitutes ``var`` with ``e`` in a symbolic matrix ``m``.

Returns:
    a matrix of symbolic expressions whose size is the size of ``m``.

Raises:
    RuntimeError if NaN is detected during substitution.)""";
      } Substitute;
      // Symbol: drake::symbolic::Substitution
      struct /* Substitution */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } Substitution;
      // Symbol: drake::symbolic::TaylorExpand
      struct /* TaylorExpand */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the Taylor series expansion of ``f`` around ``a`` of order
``order``.

Parameter ``f``:
    Symbolic expression to approximate using Taylor series expansion.

Parameter ``a``:
    Symbolic environment which specifies the point of approximation.
    If a partial environment is provided, the unspecified variables
    are treated as symbolic variables (e.g. decision variable).

Parameter ``order``:
    Positive integer which specifies the maximum order of the
    resulting polynomial approximating ``f`` around ``a``.)""";
      } TaylorExpand;
      // Symbol: drake::symbolic::UnaryExpressionCell
      struct /* UnaryExpressionCell */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc =
R"""(Represents the base class for unary expressions.)""";
        // Symbol: drake::symbolic::UnaryExpressionCell::DoEvaluate
        struct /* DoEvaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Returns the evaluation result f(``v`` ).)""";
        } DoEvaluate;
        // Symbol: drake::symbolic::UnaryExpressionCell::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::UnaryExpressionCell::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Evaluate;
        // Symbol: drake::symbolic::UnaryExpressionCell::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::UnaryExpressionCell::HashAppendDetail
        struct /* HashAppendDetail */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } HashAppendDetail;
        // Symbol: drake::symbolic::UnaryExpressionCell::Less
        struct /* Less */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""()""";
        } Less;
        // Symbol: drake::symbolic::UnaryExpressionCell::UnaryExpressionCell
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc =
R"""(Constructs UnaryExpressionCell of kind ``k`` with ``e``, ``is_poly``,
and ``is_expanded``.)""";
        } ctor;
        // Symbol: drake::symbolic::UnaryExpressionCell::get_argument
        struct /* get_argument */ {
          // Source: drake/common/symbolic/expression/expression_cell.h
          const char* doc = R"""(Returns the argument.)""";
        } get_argument;
      } UnaryExpressionCell;
      // Symbol: drake::symbolic::Variable
      struct /* Variable */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc =
R"""(Represents a symbolic variable.

Note:
    Expression∷Evaluate and Formula∷Evaluate methods take a symbolic
    environment (Variable → double) and a random number generator.
    When an expression or a formula includes random variables,
    ``Evaluate`` methods use the random number generator to draw a
    number for a random variable from the given distribution. Then
    this numeric value is used to substitute all the occurrences of
    the corresponding random variable in an expression or a formula.)""";
        // Symbol: drake::symbolic::Variable::Id
        struct /* Id */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc = R"""()""";
        } Id;
        // Symbol: drake::symbolic::Variable::Type
        struct /* Type */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc = R"""(Supported types of symbolic variables.)""";
          // Symbol: drake::symbolic::Variable::Type::BINARY
          struct /* BINARY */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(A BINARY variable takes an integer value from {0, 1}.)""";
          } BINARY;
          // Symbol: drake::symbolic::Variable::Type::BOOLEAN
          struct /* BOOLEAN */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(A BOOLEAN variable takes a ``bool`` value.)""";
          } BOOLEAN;
          // Symbol: drake::symbolic::Variable::Type::CONTINUOUS
          struct /* CONTINUOUS */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(A CONTINUOUS variable takes a ``double`` value.)""";
          } CONTINUOUS;
          // Symbol: drake::symbolic::Variable::Type::INTEGER
          struct /* INTEGER */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(An INTEGER variable takes an ``int`` value.)""";
          } INTEGER;
          // Symbol: drake::symbolic::Variable::Type::RANDOM_EXPONENTIAL
          struct /* RANDOM_EXPONENTIAL */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(A random variable whose value will be drawn from exponential
distribution with λ=1.)""";
          } RANDOM_EXPONENTIAL;
          // Symbol: drake::symbolic::Variable::Type::RANDOM_GAUSSIAN
          struct /* RANDOM_GAUSSIAN */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(A random variable whose value will be drawn from mean-zero,
unit-variance normal.)""";
          } RANDOM_GAUSSIAN;
          // Symbol: drake::symbolic::Variable::Type::RANDOM_UNIFORM
          struct /* RANDOM_UNIFORM */ {
            // Source: drake/common/symbolic/expression/variable.h
            const char* doc =
R"""(A random variable whose value will be drawn from uniform real
distributed ∈ [0,1).)""";
          } RANDOM_UNIFORM;
        } Type;
        // Symbol: drake::symbolic::Variable::Variable
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc_0args =
R"""(Constructs a default variable of type CONTINUOUS with an ``Id`` of
zero. All default-constructed variables are considered the same
variable by the equality operator (==). Similarly, a moved-from
variable is also identical to a default-constructed variable (in both
its ``name`` and its ``Id``).)""";
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc_1args =
R"""(Constructs a default value. This overload is used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc_2args =
R"""(Constructs a variable with a string. If not specified, it has
CONTINUOUS type by default.)""";
        } ctor;
        // Symbol: drake::symbolic::Variable::equal_to
        struct /* equal_to */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc =
R"""(Checks the equality of two variables based on their ID values.)""";
        } equal_to;
        // Symbol: drake::symbolic::Variable::get_id
        struct /* get_id */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc = R"""()""";
        } get_id;
        // Symbol: drake::symbolic::Variable::get_name
        struct /* get_name */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc = R"""()""";
        } get_name;
        // Symbol: drake::symbolic::Variable::get_type
        struct /* get_type */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc = R"""()""";
        } get_type;
        // Symbol: drake::symbolic::Variable::is_dummy
        struct /* is_dummy */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc =
R"""(Checks if this is the variable created by the default constructor.)""";
        } is_dummy;
        // Symbol: drake::symbolic::Variable::less
        struct /* less */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc =
R"""(Compares two variables based on their ID values.)""";
        } less;
        // Symbol: drake::symbolic::Variable::to_string
        struct /* to_string */ {
          // Source: drake/common/symbolic/expression/variable.h
          const char* doc = R"""()""";
        } to_string;
      } Variable;
      // Symbol: drake::symbolic::Variables
      struct /* Variables */ {
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc =
R"""(Represents a set of variables.

This class is based on std∷set<Variable>. The intent is to add things
that we need including set-union (Variables∷insert, operator+,
operator+=), set-minus (Variables∷erase, operator-, operator-=), and
subset/superset checking functions (Variables∷IsSubsetOf,
Variables∷IsSupersetOf, Variables∷IsStrictSubsetOf,
Variables∷IsStrictSupersetOf).)""";
        // Symbol: drake::symbolic::Variables::IsStrictSubsetOf
        struct /* IsStrictSubsetOf */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Return true if ``vars`` is a strict subset of the Variables.)""";
        } IsStrictSubsetOf;
        // Symbol: drake::symbolic::Variables::IsStrictSupersetOf
        struct /* IsStrictSupersetOf */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Return true if ``vars`` is a strict superset of the Variables.)""";
        } IsStrictSupersetOf;
        // Symbol: drake::symbolic::Variables::IsSubsetOf
        struct /* IsSubsetOf */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Return true if ``vars`` is a subset of the Variables.)""";
        } IsSubsetOf;
        // Symbol: drake::symbolic::Variables::IsSupersetOf
        struct /* IsSupersetOf */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Return true if ``vars`` is a superset of the Variables.)""";
        } IsSupersetOf;
        // Symbol: drake::symbolic::Variables::Variables
        struct /* ctor */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_0args = R"""(Default constructor.)""";
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_1args_init = R"""(List constructor.)""";
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_1args_vec =
R"""(Constructs from an Eigen vector of variables.)""";
        } ctor;
        // Symbol: drake::symbolic::Variables::begin
        struct /* begin */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Returns an iterator to the beginning.)""";
        } begin;
        // Symbol: drake::symbolic::Variables::cbegin
        struct /* cbegin */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Returns a const iterator to the beginning.)""";
        } cbegin;
        // Symbol: drake::symbolic::Variables::cend
        struct /* cend */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Returns a const iterator to the end.)""";
        } cend;
        // Symbol: drake::symbolic::Variables::const_iterator
        struct /* const_iterator */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""()""";
        } const_iterator;
        // Symbol: drake::symbolic::Variables::const_reverse_iterator
        struct /* const_reverse_iterator */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""()""";
        } const_reverse_iterator;
        // Symbol: drake::symbolic::Variables::crbegin
        struct /* crbegin */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Returns a const reverse-iterator to the beginning.)""";
        } crbegin;
        // Symbol: drake::symbolic::Variables::crend
        struct /* crend */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Returns a const reverse-iterator to the end.)""";
        } crend;
        // Symbol: drake::symbolic::Variables::empty
        struct /* empty */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Checks if this set is empty or not.)""";
        } empty;
        // Symbol: drake::symbolic::Variables::end
        struct /* end */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Returns an iterator to the end.)""";
        } end;
        // Symbol: drake::symbolic::Variables::erase
        struct /* erase */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_1args_key =
R"""(Erases ``key`` from a set. Return number of erased elements (0 or 1).)""";
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_1args_vars =
R"""(Erases variables in ``vars`` from a set. Return number of erased
elements ([0, vars.size()]).)""";
        } erase;
        // Symbol: drake::symbolic::Variables::find
        struct /* find */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Finds element with specific key.)""";
        } find;
        // Symbol: drake::symbolic::Variables::include
        struct /* include */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Return true if ``key`` is included in the Variables.)""";
        } include;
        // Symbol: drake::symbolic::Variables::insert
        struct /* insert */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_1args_var = R"""(Inserts a variable ``var`` into a set.)""";
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_2args_InputIt_InputIt =
R"""(Inserts variables in [``first``, ``last)`` into a set.)""";
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc_1args_vars =
R"""(Inserts variables in ``vars`` into a set.)""";
        } insert;
        // Symbol: drake::symbolic::Variables::iterator
        struct /* iterator */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""()""";
        } iterator;
        // Symbol: drake::symbolic::Variables::rbegin
        struct /* rbegin */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Returns a reverse iterator to the beginning.)""";
        } rbegin;
        // Symbol: drake::symbolic::Variables::rend
        struct /* rend */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Returns a reverse iterator to the end.)""";
        } rend;
        // Symbol: drake::symbolic::Variables::reverse_iterator
        struct /* reverse_iterator */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""()""";
        } reverse_iterator;
        // Symbol: drake::symbolic::Variables::size
        struct /* size */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""(Returns the number of elements.)""";
        } size;
        // Symbol: drake::symbolic::Variables::size_type
        struct /* size_type */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc = R"""()""";
        } size_type;
        // Symbol: drake::symbolic::Variables::to_string
        struct /* to_string */ {
          // Source: drake/common/symbolic/expression/variables.h
          const char* doc =
R"""(Returns string representation of Variables.)""";
        } to_string;
      } Variables;
      // Symbol: drake::symbolic::VisitExpression
      struct /* VisitExpression */ {
        // Source: drake/common/symbolic/expression/expression_visitor.h
        const char* doc =
R"""(Calls visitor object ``v`` with a symbolic-expression ``e``, and
arguments ``args``. Visitor object is expected to implement the
following methods which take ``f`` and ``args:`` ``VisitConstant``,
`VisitVariable`, ``VisitAddition``, `VisitMultiplication`,
``VisitDivision``, `VisitLog`, ``VisitAbs``, `VisitExp`,
``VisitSqrt``, `VisitPow`, ``VisitSin``, `VisitCos`, ``VisitTan``,
`VisitAsin`, ``VisitAtan``, `VisitAtan2`, ``VisitSinh``, `VisitCosh`,
``VisitTanh``, `VisitMin`, ``VisitMax``, `VisitCeil`, ``VisitFloor``,
`VisitIfThenElse`, ``VisitUninterpretedFunction``.

Raises:
    RuntimeError if NaN is detected during a visit.)""";
      } VisitExpression;
      // Symbol: drake::symbolic::VisitFormula
      struct /* VisitFormula */ {
        // Source: drake/common/symbolic/expression/formula_visitor.h
        const char* doc =
R"""(Calls visitor object ``v`` with a symbolic formula ``f``, and
arguments ``args``. Visitor object is expected to implement the
following methods which take ``f`` and ``args:`` ``VisitFalse``,
`VisitTrue`, ``VisitVariable``, `VisitEqualTo`, VisitNotEqualTo,
VisitGreaterThan, ``VisitGreaterThanOrEqualTo``, `VisitLessThan`,
``VisitLessThanOrEqualTo``, `VisitConjunction`, ``VisitDisjunction``,
`VisitNegation`, ``VisitForall``, `VisitIsnan`,
``VisitPositiveSemidefinite``.

Check the implementation of ``NegationNormalFormConverter`` class in
drake/common/test/symbolic_formula_visitor_test.cc file to find an
example.)""";
      } VisitFormula;
      // Symbol: drake::symbolic::VisitPolynomial
      struct /* VisitPolynomial */ {
        // Source: drake/common/symbolic/expression/expression_visitor.h
        const char* doc =
R"""(Calls visitor object ``v`` with a polynomial symbolic-expression
``e``, and arguments ``args``. Visitor object is expected to implement
the following methods which take ``f`` and ``args:``
``VisitConstant``, `VisitVariable`, ``VisitAddition``,
`VisitMultiplication`, ``VisitDivision``, `VisitPow`.

Raises:
    RuntimeError if NaN is detected during a visit.

See the implementation of ``DegreeVisitor`` class and ``Degree``
function in drake/common/symbolic_monomial.cc as an example usage.

Precondition:
    e.is_polynomial() is true.)""";
      } VisitPolynomial;
      // Symbol: drake::symbolic::abs
      struct /* abs */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } abs;
      // Symbol: drake::symbolic::acos
      struct /* acos */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } acos;
      // Symbol: drake::symbolic::asin
      struct /* asin */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } asin;
      // Symbol: drake::symbolic::atan
      struct /* atan */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } atan;
      // Symbol: drake::symbolic::atan2
      struct /* atan2 */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } atan2;
      // Symbol: drake::symbolic::ceil
      struct /* ceil */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } ceil;
      // Symbol: drake::symbolic::clamp
      struct /* clamp */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } clamp;
      // Symbol: drake::symbolic::cos
      struct /* cos */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } cos;
      // Symbol: drake::symbolic::cosh
      struct /* cosh */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } cosh;
      // Symbol: drake::symbolic::exp
      struct /* exp */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } exp;
      // Symbol: drake::symbolic::floor
      struct /* floor */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } floor;
      // Symbol: drake::symbolic::forall
      struct /* forall */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns a formula ``f``, universally quantified by variables ``vars``.)""";
      } forall;
      // Symbol: drake::symbolic::get_argument
      struct /* get_argument */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the argument in the unary expression ``e``.

Precondition:
    {``e`` is a unary expression.})""";
      } get_argument;
      // Symbol: drake::symbolic::get_base_to_exponent_map_in_multiplication
      struct /* get_base_to_exponent_map_in_multiplication */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the map from a base expression to its exponent expression in
the multiplication expression ``e``. For instance, given 7 * x^2 * y^3
* z^x, the return value maps 'x' to 2, 'y' to 3, and 'z' to 'x'.

Precondition:
    {``e`` is a multiplication expression.})""";
      } get_base_to_exponent_map_in_multiplication;
      // Symbol: drake::symbolic::get_conditional_formula
      struct /* get_conditional_formula */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the conditional formula in the if-then-else expression ``e``.

Precondition:
    ``e`` is an if-then-else expression.)""";
      } get_conditional_formula;
      // Symbol: drake::symbolic::get_constant_in_addition
      struct /* get_constant_in_addition */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the constant part of the addition expression ``e``. For
instance, given 7 + 2 * x + 3 * y, it returns 7.

Precondition:
    {``e`` is an addition expression.})""";
      } get_constant_in_addition;
      // Symbol: drake::symbolic::get_constant_in_multiplication
      struct /* get_constant_in_multiplication */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the constant part of the multiplication expression ``e``. For
instance, given 7 * x^2 * y^3, it returns 7.

Precondition:
    {``e`` is a multiplication expression.})""";
      } get_constant_in_multiplication;
      // Symbol: drake::symbolic::get_constant_value
      struct /* get_constant_value */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the constant value of the constant expression ``e``.

Precondition:
    {``e`` is a constant expression.})""";
      } get_constant_value;
      // Symbol: drake::symbolic::get_else_expression
      struct /* get_else_expression */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the 'else' expression in the if-then-else expression ``e``.

Precondition:
    ``e`` is an if-then-else expression.)""";
      } get_else_expression;
      // Symbol: drake::symbolic::get_expr_to_coeff_map_in_addition
      struct /* get_expr_to_coeff_map_in_addition */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the map from an expression to its coefficient in the addition
expression ``e``. For instance, given 7 + 2 * x + 3 * y, the return
value maps 'x' to 2 and 'y' to 3.

Precondition:
    {``e`` is an addition expression.})""";
      } get_expr_to_coeff_map_in_addition;
      // Symbol: drake::symbolic::get_first_argument
      struct /* get_first_argument */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the first argument of the binary expression ``e``.

Precondition:
    {``e`` is a binary expression.})""";
      } get_first_argument;
      // Symbol: drake::symbolic::get_lhs_expression
      struct /* get_lhs_expression */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the lhs-argument of a relational formula ``f``.

Precondition:
    {``f`` is a relational formula.})""";
      } get_lhs_expression;
      // Symbol: drake::symbolic::get_matrix_in_positive_semidefinite
      struct /* get_matrix_in_positive_semidefinite */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the matrix in a positive-semidefinite formula ``f``.

Precondition:
    {``f`` is a positive-semidefinite formula.})""";
      } get_matrix_in_positive_semidefinite;
      // Symbol: drake::symbolic::get_operand
      struct /* get_operand */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the formula in a negation formula ``f``.

Precondition:
    {``f`` is a negation formula.})""";
      } get_operand;
      // Symbol: drake::symbolic::get_operands
      struct /* get_operands */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the set of formulas in a n-ary formula ``f``.

Precondition:
    {``f`` is a n-ary formula.})""";
      } get_operands;
      // Symbol: drake::symbolic::get_quantified_formula
      struct /* get_quantified_formula */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the quantified formula in a forall formula ``f``.

Precondition:
    {``f`` is a forall formula.})""";
      } get_quantified_formula;
      // Symbol: drake::symbolic::get_quantified_variables
      struct /* get_quantified_variables */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the quantified variables in a forall formula ``f``.

Precondition:
    {``f`` is a forall formula.})""";
      } get_quantified_variables;
      // Symbol: drake::symbolic::get_rhs_expression
      struct /* get_rhs_expression */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the rhs-argument of a relational formula ``f``.

Precondition:
    {``f`` is a relational formula.})""";
      } get_rhs_expression;
      // Symbol: drake::symbolic::get_second_argument
      struct /* get_second_argument */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the second argument of the binary expression ``e``.

Precondition:
    {``e`` is a binary expression.})""";
      } get_second_argument;
      // Symbol: drake::symbolic::get_then_expression
      struct /* get_then_expression */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the 'then' expression in the if-then-else expression ``e``.

Precondition:
    ``e`` is an if-then-else expression.)""";
      } get_then_expression;
      // Symbol: drake::symbolic::get_unary_expression
      struct /* get_unary_expression */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns the expression in a unary expression formula ``f``. Currently,
an isnan() formula is the only kind of unary expression formula.

Precondition:
    {``f`` is a unary expression formula.})""";
      } get_unary_expression;
      // Symbol: drake::symbolic::get_uninterpreted_function_arguments
      struct /* get_uninterpreted_function_arguments */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the arguments of an uninterpreted-function expression ``e``.

Precondition:
    ``e`` is an uninterpreted-function expression.)""";
      } get_uninterpreted_function_arguments;
      // Symbol: drake::symbolic::get_uninterpreted_function_name
      struct /* get_uninterpreted_function_name */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Returns the name of an uninterpreted-function expression ``e``.

Precondition:
    ``e`` is an uninterpreted-function expression.)""";
      } get_uninterpreted_function_name;
      // Symbol: drake::symbolic::get_variable
      struct /* get_variable */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Returns the embedded variable in the variable expression ``e``.

Precondition:
    {``e`` is a variable expression.})""";
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_1args_f =
R"""(Returns the embedded variable in the variable formula ``f``.

Precondition:
    ``f`` is a variable formula.)""";
      } get_variable;
      // Symbol: drake::symbolic::if_then_else
      struct /* if_then_else */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Constructs if-then-else expression.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    if_then_else(cond, expr_then, expr_else)

.. raw:: html

    </details>

The value returned by the above if-then-else expression is
``expr_then`` if ``cond`` is evaluated to true. Otherwise, it returns
``expr_else``.

The semantics is similar to the C++'s conditional expression
constructed by its ternary operator, @c ?:. However, there is a key
difference between the C++'s conditional expression and our
``if_then_else`` expression in a way the arguments are evaluated
during the construction.

- In case of the C++'s conditional expression, `` cond ? expr_then :
expr_else``, the then expression ``expr_then`` (respectively, the else
expression ``expr_else)`` is **only** evaluated when the conditional
expression ``cond`` is evaluated to **true** (respectively, when ``cond`` is
evaluated to **false)**.

- In case of the symbolic expression, ``if_then_else(cond, expr_then,
expr_else)``, however, **both** arguments ``expr_then`` and ``expr_else``
are evaluated first and then passed to the ``if_then_else`` function.

Note:
    This function returns an **expression** and it is different from
    the C++'s if-then-else **statement**.

Note:
    While it is still possible to define `` min, max, abs`` math
    functions using ``if_then_else`` expression, it is highly
    **recommended** to use the provided native definitions for them
    because it allows solvers to detect specific math functions and to
    have a room for special optimizations.

Note:
    More information about the C++'s conditional expression and
    ternary operator is available at
    http://en.cppreference.com/w/cpp/language/operator_other#Conditional_operator.)""";
      } if_then_else;
      // Symbol: drake::symbolic::intersect
      struct /* intersect */ {
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc =
R"""(Returns the intersection of ``vars1`` and ``vars2``.

This function has a time complexity of ``O(N₁ + N₂)`` where ``N₁`` and
``N₂`` are the size of ``vars1`` and ``vars2`` respectively.)""";
      } intersect;
      // Symbol: drake::symbolic::is_abs
      struct /* is_abs */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is an abs expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an absolute-value-function expression.)""";
      } is_abs;
      // Symbol: drake::symbolic::is_acos
      struct /* is_acos */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an arccosine expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an arccosine expression.)""";
      } is_acos;
      // Symbol: drake::symbolic::is_addition
      struct /* is_addition */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an addition expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an addition expression.)""";
      } is_addition;
      // Symbol: drake::symbolic::is_asin
      struct /* is_asin */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an arcsine expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an arcsine expression.)""";
      } is_asin;
      // Symbol: drake::symbolic::is_atan
      struct /* is_atan */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an arctangent expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an arctangent expression.)""";
      } is_atan;
      // Symbol: drake::symbolic::is_atan2
      struct /* is_atan2 */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an arctangent2 expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a arctangent2 expression.)""";
      } is_atan2;
      // Symbol: drake::symbolic::is_binary
      struct /* is_binary */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc = R"""(Checks if ``c`` is a binary expression.)""";
      } is_binary;
      // Symbol: drake::symbolic::is_ceil
      struct /* is_ceil */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a ceil expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a ceil expression.)""";
      } is_ceil;
      // Symbol: drake::symbolic::is_conjunction
      struct /* is_conjunction */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""(Checks if ``f`` is a conjunction (∧).)""";
      } is_conjunction;
      // Symbol: drake::symbolic::is_constant
      struct /* is_constant */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args =
R"""(Checks if ``e`` is a constant expression.)""";
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_2args =
R"""(Checks if ``e`` is a constant expression representing ``v``.)""";
      } is_constant;
      // Symbol: drake::symbolic::is_cos
      struct /* is_cos */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a cosine expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a cosine expression.)""";
      } is_cos;
      // Symbol: drake::symbolic::is_cosh
      struct /* is_cosh */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a hyperbolic-cosine expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a hyperbolic-cosine expression.)""";
      } is_cosh;
      // Symbol: drake::symbolic::is_disjunction
      struct /* is_disjunction */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""(Checks if ``f`` is a disjunction (∨).)""";
      } is_disjunction;
      // Symbol: drake::symbolic::is_division
      struct /* is_division */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a division expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a division expression.)""";
      } is_division;
      // Symbol: drake::symbolic::is_equal_to
      struct /* is_equal_to */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a formula representing equality (==).)""";
      } is_equal_to;
      // Symbol: drake::symbolic::is_exp
      struct /* is_exp */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is an exp expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is an exp expression.)""";
      } is_exp;
      // Symbol: drake::symbolic::is_false
      struct /* is_false */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is structurally equal to False formula.)""";
      } is_false;
      // Symbol: drake::symbolic::is_floor
      struct /* is_floor */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a floor expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a floor expression.)""";
      } is_floor;
      // Symbol: drake::symbolic::is_forall
      struct /* is_forall */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a Forall formula (∀).)""";
      } is_forall;
      // Symbol: drake::symbolic::is_greater_than
      struct /* is_greater_than */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a formula representing greater-than (>).)""";
      } is_greater_than;
      // Symbol: drake::symbolic::is_greater_than_or_equal_to
      struct /* is_greater_than_or_equal_to */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a formula representing greater-than-or-equal-to
(>=).)""";
      } is_greater_than_or_equal_to;
      // Symbol: drake::symbolic::is_if_then_else
      struct /* is_if_then_else */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an if-then-else expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an if-then-else expression.)""";
      } is_if_then_else;
      // Symbol: drake::symbolic::is_integer
      struct /* is_integer */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc = R"""()""";
      } is_integer;
      // Symbol: drake::symbolic::is_isnan
      struct /* is_isnan */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""(Checks if ``f`` is an isnan formula.)""";
      } is_isnan;
      // Symbol: drake::symbolic::is_less_than
      struct /* is_less_than */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a formula representing less-than (<).)""";
      } is_less_than;
      // Symbol: drake::symbolic::is_less_than_or_equal_to
      struct /* is_less_than_or_equal_to */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a formula representing less-than-or-equal-to (<=).)""";
      } is_less_than_or_equal_to;
      // Symbol: drake::symbolic::is_log
      struct /* is_log */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a log expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a log expression.)""";
      } is_log;
      // Symbol: drake::symbolic::is_max
      struct /* is_max */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a max expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a max expression.)""";
      } is_max;
      // Symbol: drake::symbolic::is_min
      struct /* is_min */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a min expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a min expression.)""";
      } is_min;
      // Symbol: drake::symbolic::is_multiplication
      struct /* is_multiplication */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a multiplication expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an multiplication expression.)""";
      } is_multiplication;
      // Symbol: drake::symbolic::is_nan
      struct /* is_nan */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""(Checks if ``e`` is NaN.)""";
      } is_nan;
      // Symbol: drake::symbolic::is_nary
      struct /* is_nary */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Checks if ``f`` is a n-ary formula ({∧, ∨}).)""";
      } is_nary;
      // Symbol: drake::symbolic::is_neg_one
      struct /* is_neg_one */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""(Checks if ``e`` is -1.0.)""";
      } is_neg_one;
      // Symbol: drake::symbolic::is_negation
      struct /* is_negation */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""(Checks if ``f`` is a negation (¬).)""";
      } is_negation;
      // Symbol: drake::symbolic::is_non_negative_integer
      struct /* is_non_negative_integer */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc = R"""()""";
      } is_non_negative_integer;
      // Symbol: drake::symbolic::is_not_equal_to
      struct /* is_not_equal_to */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a formula representing disequality (!=).)""";
      } is_not_equal_to;
      // Symbol: drake::symbolic::is_one
      struct /* is_one */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""(Checks if ``e`` is 1.0.)""";
      } is_one;
      // Symbol: drake::symbolic::is_positive_integer
      struct /* is_positive_integer */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc = R"""()""";
      } is_positive_integer;
      // Symbol: drake::symbolic::is_positive_semidefinite
      struct /* is_positive_semidefinite */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Checks if ``f`` is a positive-semidefinite formula.)""";
      } is_positive_semidefinite;
      // Symbol: drake::symbolic::is_pow
      struct /* is_pow */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a power-function expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a power-function expression.)""";
      } is_pow;
      // Symbol: drake::symbolic::is_relational
      struct /* is_relational */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is a relational formula ({==, !=, >, >=, <, <=}).)""";
      } is_relational;
      // Symbol: drake::symbolic::is_sin
      struct /* is_sin */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Checks if ``e`` is a sine expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c = R"""(Checks if ``c`` is a sine expression.)""";
      } is_sin;
      // Symbol: drake::symbolic::is_sinh
      struct /* is_sinh */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a hyperbolic-sine expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a hyperbolic-sine expression.)""";
      } is_sinh;
      // Symbol: drake::symbolic::is_sqrt
      struct /* is_sqrt */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a square-root expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a square-root expression.)""";
      } is_sqrt;
      // Symbol: drake::symbolic::is_tan
      struct /* is_tan */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a tangent expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a tangent expression.)""";
      } is_tan;
      // Symbol: drake::symbolic::is_tanh
      struct /* is_tanh */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a hyperbolic-tangent expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a hyperbolic-tangent expression.)""";
      } is_tanh;
      // Symbol: drake::symbolic::is_true
      struct /* is_true */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Checks if ``f`` is structurally equal to True formula.)""";
      } is_true;
      // Symbol: drake::symbolic::is_two
      struct /* is_two */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""(Checks if ``e`` is 2.0.)""";
      } is_two;
      // Symbol: drake::symbolic::is_unary
      struct /* is_unary */ {
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc = R"""(Checks if ``c`` is a unary expression.)""";
      } is_unary;
      // Symbol: drake::symbolic::is_uninterpreted_function
      struct /* is_uninterpreted_function */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is an uninterpreted-function expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is an uninterpreted-function expression.)""";
      } is_uninterpreted_function;
      // Symbol: drake::symbolic::is_variable
      struct /* is_variable */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e =
R"""(Checks if ``e`` is a variable expression.)""";
        // Source: drake/common/symbolic/expression/expression_cell.h
        const char* doc_1args_c =
R"""(Checks if ``c`` is a variable expression.)""";
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_1args_f = R"""(Checks if ``f`` is a variable formula.)""";
      } is_variable;
      // Symbol: drake::symbolic::is_zero
      struct /* is_zero */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""(Checks if ``e`` is 0.0.)""";
      } is_zero;
      // Symbol: drake::symbolic::isfinite
      struct /* isfinite */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns a Formula determining if the given expression ``e`` has a
finite value.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
      } isfinite;
      // Symbol: drake::symbolic::isinf
      struct /* isinf */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns a Formula determining if the given expression ``e`` is a
positive or negative infinity.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
      } isinf;
      // Symbol: drake::symbolic::isnan
      struct /* isnan */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns a Formula for the predicate isnan(e) to the given expression.
This serves as the argument-dependent lookup related to
std∷isnan(double).

When this formula is evaluated, there are two possible outcomes: -
Returns false if the e.Evaluate() is not NaN. - Throws RuntimeError if
NaN is detected during evaluation. Note that the evaluation of
``isnan(e)`` never returns true.)""";
      } isnan;
      // Symbol: drake::symbolic::log
      struct /* log */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } log;
      // Symbol: drake::symbolic::make_conjunction
      struct /* make_conjunction */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns a conjunction of ``formulas``. It performs the following
simplification:

- make_conjunction({}) returns True.
- make_conjunction({f₁}) returns f₁.
- If False ∈ ``formulas``, it returns False.
- If True ∈ ``formulas``, it will not appear in the return value.
- Nested conjunctions will be flattened. For example, make_conjunction({f₁,
  f₂ ∧ f₃}) returns f₁ ∧ f₂ ∧ f₃.)""";
      } make_conjunction;
      // Symbol: drake::symbolic::make_disjunction
      struct /* make_disjunction */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc =
R"""(Returns a disjunction of ``formulas``. It performs the following
simplification:

- make_disjunction({}) returns False.
- make_disjunction({f₁}) returns f₁.
- If True ∈ ``formulas``, it returns True.
- If False ∈ ``formulas``, it will not appear in the return value.
- Nested disjunctions will be flattened. For example, make_disjunction({f₁,
  f₂ ∨ f₃}) returns f₁ ∨ f₂ ∨ f₃.)""";
      } make_disjunction;
      // Symbol: drake::symbolic::max
      struct /* max */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } max;
      // Symbol: drake::symbolic::min
      struct /* min */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } min;
      // Symbol: drake::symbolic::operator!
      struct /* operator_lnot */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""()""";
      } operator_lnot;
      // Symbol: drake::symbolic::operator!=
      struct /* operator_ne */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } operator_ne;
      // Symbol: drake::symbolic::operator&&
      struct /* operator_land */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""()""";
      } operator_land;
      // Symbol: drake::symbolic::operator*
      struct /* operator_mul */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } operator_mul;
      // Symbol: drake::symbolic::operator*=
      struct /* operator_imul */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } operator_imul;
      // Symbol: drake::symbolic::operator+
      struct /* operator_add */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Provides unary plus operator.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars1_vars2 =
R"""(Returns set-union of ``var1`` and ``var2``.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars_var =
R"""(Returns set-union of ``vars`` and {``var``}.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_var_vars =
R"""(Returns set-union of {``var``} and ``vars``.)""";
      } operator_add;
      // Symbol: drake::symbolic::operator+=
      struct /* operator_iadd */ {
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars1_vars2 =
R"""(Updates ``var1`` with the result of set-union(``var1``, ``var2)``.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars_var =
R"""(Updates ``vars`` with the result of set-union(``vars``, { ``var`` }).)""";
      } operator_iadd;
      // Symbol: drake::symbolic::operator-
      struct /* operator_sub */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc_1args_e = R"""(Provides unary minus operator.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars1_vars2 = R"""(Returns set-minus(``var1``, ``vars2)``.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars_var =
R"""(Returns set-minus(``vars``, { ``var`` }).)""";
      } operator_sub;
      // Symbol: drake::symbolic::operator-=
      struct /* operator_isub */ {
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars1_vars2 =
R"""(Updates ``var1`` with the result of set-minus(``var1``, ``var2)``.)""";
        // Source: drake/common/symbolic/expression/variables.h
        const char* doc_2args_vars_var =
R"""(Updates ``vars`` with the result of set-minus(``vars``, {``var``}).)""";
      } operator_isub;
      // Symbol: drake::symbolic::operator/
      struct /* operator_div */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } operator_div;
      // Symbol: drake::symbolic::operator/=
      struct /* operator_idiv */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } operator_idiv;
      // Symbol: drake::symbolic::operator<
      struct /* operator_lt */ {
        // Source: drake/common/symbolic/expression/expression_kind.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""(Total ordering between ExpressionKinds.)""";
      } operator_lt;
      // Symbol: drake::symbolic::operator<=
      struct /* operator_le */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } operator_le;
      // Symbol: drake::symbolic::operator>
      struct /* operator_gt */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } operator_gt;
      // Symbol: drake::symbolic::operator>=
      struct /* operator_ge */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
      } operator_ge;
      // Symbol: drake::symbolic::operator||
      struct /* operator_lor */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc = R"""()""";
      } operator_lor;
      // Symbol: drake::symbolic::positive_semidefinite
      struct /* positive_semidefinite */ {
        // Source: drake/common/symbolic/expression/formula.h
        const char* doc_1args_m =
R"""(Returns a symbolic formula constraining ``m`` to be a
positive-semidefinite matrix. By definition, a symmetric matrix ``m``
is positive-semidefinte if xᵀ m x ≥ 0 for all vector x ∈ ℝⁿ.

Raises:
    RuntimeError if ``m`` is not symmetric.

Note:
    This method checks if ``m`` is symmetric, which can be costly. If
    you want to avoid it, please consider using
    ``positive_semidefinite(m.triangularView<Eigen∷Lower>())`` or
    ``positive_semidefinite(m.triangularView<Eigen∷Upper>())`` instead
    of ``positive_semidefinite(m)``.)""";
      } positive_semidefinite;
      // Symbol: drake::symbolic::pow
      struct /* pow */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } pow;
      // Symbol: drake::symbolic::sin
      struct /* sin */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } sin;
      // Symbol: drake::symbolic::sinh
      struct /* sinh */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } sinh;
      // Symbol: drake::symbolic::sqrt
      struct /* sqrt */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } sqrt;
      // Symbol: drake::symbolic::swap
      struct /* swap */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } swap;
      // Symbol: drake::symbolic::tan
      struct /* tan */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } tan;
      // Symbol: drake::symbolic::tanh
      struct /* tanh */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc = R"""()""";
      } tanh;
      // Symbol: drake::symbolic::to_conjunction
      struct /* to_conjunction */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaAnd>.

Precondition:
    ``is_conjunction(*f_ptr)`` is true.)""";
      } to_conjunction;
      // Symbol: drake::symbolic::to_disjunction
      struct /* to_disjunction */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaOr>.

Precondition:
    ``is_disjunction(*f_ptr)`` is true.)""";
      } to_disjunction;
      // Symbol: drake::symbolic::to_equal_to
      struct /* to_equal_to */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaEq>.

Precondition:
    ``is_equal_to(*f_ptr)`` is true.)""";
      } to_equal_to;
      // Symbol: drake::symbolic::to_false
      struct /* to_false */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaFalse>.

Precondition:
    ``is_false(*f_ptr)`` is true.)""";
      } to_false;
      // Symbol: drake::symbolic::to_forall
      struct /* to_forall */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaForall>.

Precondition:
    ``is_forall(*f_ptr)`` is true.)""";
      } to_forall;
      // Symbol: drake::symbolic::to_greater_than
      struct /* to_greater_than */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaGt>.

Precondition:
    ``is_greater_than(*f_ptr)`` is true.)""";
      } to_greater_than;
      // Symbol: drake::symbolic::to_greater_than_or_equal_to
      struct /* to_greater_than_or_equal_to */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaGeq>.

Precondition:
    ``is_greater_than_or_equal_to(*f_ptr)`` is true.)""";
      } to_greater_than_or_equal_to;
      // Symbol: drake::symbolic::to_isnan
      struct /* to_isnan */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaIsnan>.

Precondition:
    ``is_isnan(*f_ptr)`` is true.)""";
      } to_isnan;
      // Symbol: drake::symbolic::to_less_than
      struct /* to_less_than */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaLt>.

Precondition:
    ``is_less_than(*f_ptr)`` is true.)""";
      } to_less_than;
      // Symbol: drake::symbolic::to_less_than_or_equal_to
      struct /* to_less_than_or_equal_to */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaLeq>.

Precondition:
    ``is_less_than_or_equal_to(*f_ptr)`` is true.)""";
      } to_less_than_or_equal_to;
      // Symbol: drake::symbolic::to_nary
      struct /* to_nary */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const NaryFormulaCell>.

Precondition:
    ``is_nary(*f_ptr)`` is true.)""";
      } to_nary;
      // Symbol: drake::symbolic::to_negation
      struct /* to_negation */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaNot>.

Precondition:
    ``is_negation(*f_ptr)`` is true.)""";
      } to_negation;
      // Symbol: drake::symbolic::to_not_equal_to
      struct /* to_not_equal_to */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaNeq>.

Precondition:
    ``is_not_equal_to(*f_ptr)`` is true.)""";
      } to_not_equal_to;
      // Symbol: drake::symbolic::to_positive_semidefinite
      struct /* to_positive_semidefinite */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaPositiveSemidefinite>.

Precondition:
    ``is_positive_semidefinite(*f_ptr)`` is true.)""";
      } to_positive_semidefinite;
      // Symbol: drake::symbolic::to_relational
      struct /* to_relational */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const RelationalFormulaCell>.

Precondition:
    ``is_relational(*f_ptr)`` is true.)""";
      } to_relational;
      // Symbol: drake::symbolic::to_string
      struct /* to_string */ {
        // Source: drake/common/symbolic/expression/variable.h
        const char* doc = R"""()""";
      } to_string;
      // Symbol: drake::symbolic::to_true
      struct /* to_true */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaTrue>.

Precondition:
    ``is_true(*f_ptr)`` is true.)""";
      } to_true;
      // Symbol: drake::symbolic::to_variable
      struct /* to_variable */ {
        // Source: drake/common/symbolic/expression/formula_cell.h
        const char* doc =
R"""(Casts ``f_ptr`` to ``shared_ptr``<const FormulaVar>.

Precondition:
    ``is_variable(*f_ptr)`` is true.)""";
      } to_variable;
      // Symbol: drake::symbolic::uninterpreted_function
      struct /* uninterpreted_function */ {
        // Source: drake/common/symbolic/expression/expression.h
        const char* doc =
R"""(Constructs an uninterpreted-function expression with ``name`` and
``arguments``. An uninterpreted function is an opaque function that
has no other property than its name and a list of its arguments. This
is useful to applications where it is good enough to provide abstract
information of a function without exposing full details. Declaring
sparsity of a system is a typical example.)""";
      } uninterpreted_function;
    } symbolic;
  } drake;
} pydrake_doc_common_symbolic_expression;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
