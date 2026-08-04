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

// #include "drake/common/symbolic/chebyshev_basis_element.h"
// #include "drake/common/symbolic/chebyshev_polynomial.h"
// #include "drake/common/symbolic/codegen.h"
// #include "drake/common/symbolic/decompose.h"
// #include "drake/common/symbolic/expression.h"
// #include "drake/common/symbolic/generic_polynomial.h"
// #include "drake/common/symbolic/latex.h"
// #include "drake/common/symbolic/monomial.h"
// #include "drake/common/symbolic/monomial_basis_element.h"
// #include "drake/common/symbolic/monomial_util.h"
// #include "drake/common/symbolic/polynomial.h"
// #include "drake/common/symbolic/polynomial_basis.h"
// #include "drake/common/symbolic/polynomial_basis_element.h"
// #include "drake/common/symbolic/rational_function.h"
// #include "drake/common/symbolic/replace_bilinear_terms.h"
// #include "drake/common/symbolic/simplification.h"
// #include "drake/common/symbolic/trigonometric_polynomial.h"

// Symbol: pydrake_doc_common_symbolic
constexpr struct /* pydrake_doc_common_symbolic */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::symbolic
    struct /* symbolic */ {
      // Symbol: drake::symbolic::BasisElementGradedReverseLexOrder
      struct /* BasisElementGradedReverseLexOrder */ {
        // Source: drake/common/symbolic/polynomial_basis_element.h
        const char* doc =
R"""(Implements Graded reverse lexicographic order.

Template parameter ``VariableOrder``:
    VariableOrder{}(v1, v2) is true if v1 < v2.

Template parameter ``BasisElement``:
    A derived class of PolynomialBasisElement.

We first compare the total degree of the PolynomialBasisElement; if
there is a tie, then we use the graded reverse lexicographical order
as the tie breaker.

Take monomials with variables {x, y, z} and total degree<=2 as an
example, with the order x > y > z. To get the graded reverse
lexicographical order, we take the following steps:

First find all the monomials using the total degree. The monomials
with degree 2 are {x², y², z², xy, xz, yz}. The monomials with degree
1 are {x, y, z}, and the monomials with degree 0 is {1}. To break the
tie between monomials with the same total degree, first sort them in
the reverse lexicographical order, namely x < y < z. The
lexicographical order compares two monomials by first comparing the
exponent of the largest variable, if there is a tie then go forth to
the second largest variable. Thus z² > zy >zx > y² > yx > x². Finally
reverse the order as x² > xy > y² > xz > yz > z² > x > y > z.

There is an introduction to monomial order in
https://en.wikipedia.org/wiki/Monomial_order, and an introduction to
graded reverse lexicographical order in
https://en.wikipedia.org/wiki/Monomial_order#Graded_reverse_lexicographic_order)""";
        // Symbol: drake::symbolic::BasisElementGradedReverseLexOrder::operator()
        struct /* operator_call */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Returns true if m1 < m2 under the Graded reverse lexicographic order.)""";
        } operator_call;
      } BasisElementGradedReverseLexOrder;
      // Symbol: drake::symbolic::CalcMonomialBasisOrderUpToOne
      struct /* CalcMonomialBasisOrderUpToOne */ {
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc =
R"""(Generates all the monomials of ``x``, such that the degree for x(i) is
no larger than 1 for every x(i) in ``x``.

Parameter ``x``:
    The variables whose monomials are generated.

Parameter ``sort_monomial``:
    If true, the returned monomials are sorted in the graded reverse
    lexicographic order. For example if x = (x₀, x₁) with x₀< x₁, then
    this function returns [x₀x₁, x₁, x₀, 1]. If sort_monomial=false,
    then we return the monomials in an arbitrary order.)""";
      } CalcMonomialBasisOrderUpToOne;
      // Symbol: drake::symbolic::CalcPolynomialWLowerTriangularPart
      struct /* CalcPolynomialWLowerTriangularPart */ {
        // Source: drake/common/symbolic/polynomial.h
        const char* doc =
R"""(Returns the polynomial m(x)ᵀ * Q * m(x), where m(x) is the monomial
basis, and Q is the Gram matrix.

Parameter ``monomial_basis``:
    m(x) in the documentation. A vector of monomials.

Parameter ``gram_lower``:
    The lower triangular entries in Q, stacked columnwise into a
    vector.)""";
      } CalcPolynomialWLowerTriangularPart;
      // Symbol: drake::symbolic::ChebyshevBasisElement
      struct /* ChebyshevBasisElement */ {
        // Source: drake/common/symbolic/chebyshev_basis_element.h
        const char* doc =
R"""(ChebyshevBasisElement represents an element of Chebyshev polynomial
basis, written as the product of Chebyshev polynomials, in the form
Tₚ₀(x₀)Tₚ₁(x₁)...Tₚₙ(xₙ), where each Tₚᵢ(xᵢ) is a (univariate)
Chebyshev polynomial of degree pᵢ.)""";
        // Symbol: drake::symbolic::ChebyshevBasisElement::ChebyshevBasisElement
        struct /* ctor */ {
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc_0args =
R"""(Constructs a ChebyshevBasisElement equals to 1.)""";
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc_1args_var =
R"""(Constructs a Chebyshev polynomial T₁(var).)""";
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc_2args_var_degree =
R"""(Constructs a Chebyshev polynomial Tₙ(var) where n = degree.)""";
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc_1args_stdnullptrt =
R"""(Constructs a default value 1. This overload is used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
        } ctor;
        // Symbol: drake::symbolic::ChebyshevBasisElement::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc =
R"""(Differentiates the ChebyshevBasisElement with respect to a variable.
We use the fact that - If n is even dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x), j is odd
and 1 <= j <= n-1 - If n is odd dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x) - n, j is even
and 0 <= j <= n-1 We return ``result``, a map from
ChebyshevBasisElement to double, such that sum(result.key() *
result[key]) is the differentiation of ``this`` w.r.t the variable.
For example if n is even, dTₙ(x)Tₘ(y)/dx = 2n∑ⱼ Tⱼ(x)Tₘ(y), j is odd
and 1 <= j <= n-1, then the returned result is {T₁(x)Tₘ(y), 2n},
{T₃(x)Tₘ(y), 2n}, ..., {T₂ₙ₋₁(x)Tₘ(y), 2n}. A special case is that
``var`` is not a variable in ``this``, then we return an empty map.

Parameter ``var``:
    A variable to differentiate with.)""";
        } Differentiate;
        // Symbol: drake::symbolic::ChebyshevBasisElement::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc =
R"""(Partially evaluates using a given environment ``env``. The evaluation
result is of type pair<double, ChebyshevBasisElement>. The first
component (: double) represents the coefficient part while the second
component represents the remaining parts of the ChebyshevBasisElement
which was not evaluated, the product of the first and the second
component is the result of the partial evaluation. For example, if
this ChebyshevBasisElement is T₂(x)T₃(y)T₁(z), and ``env`` stores x→
3, y→ 2, then the partial evaluation is T₂(3)*T₃(2)*T₁(z) = 17 * 26 *
T₁(z) = 442*T₁(z), then we return the pair (442, T₁(z)).)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::ChebyshevBasisElement::Integrate
        struct /* Integrate */ {
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc =
R"""(Integrates a ChebyshevBasisElement for a variable. We use the fact
that ∫ Tₙ(x)dx = 1/(2n+2)Tₙ₊₁(x) − 1/(2n−2)Tₙ₋₁(x) A special case is ∫
T₀(x)dx = T₁(x)

Parameter ``var``:
    The variable to integrate. If ``var`` is not a variable in this
    ChebyshevBasisElement, then the integration result is *this *
    T₁(var).

Returns ``result``:
    sum(key * result[key]) is the integration result. For example, ∫
    T₂(x)T₃(y)dx = 1/6*T₃(x)T₃(y) − 1/2 * T₁(x)T₃(y), then the result
    is the map containing {T₃(x)T₃(y), 1/6} and {T₁(x)T₃(y), -1/2}.)""";
        } Integrate;
        // Symbol: drake::symbolic::ChebyshevBasisElement::MergeBasisElementInPlace
        struct /* MergeBasisElementInPlace */ {
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc =
R"""(Merges this Chebyshev basis element with another Chebyshev basis
element ``other`` by merging their var_to_degree_map. After merging,
the degree of each variable is raised to the sum of the degree in each
basis element (if a variable does not show up in either one of the
basis element, we regard its degree to be 0). For example, merging
T₁(x)T₃(y) and T₂(x)T₄(z) gets T₃(x)T₃(y)T₄(z).)""";
        } MergeBasisElementInPlace;
        // Symbol: drake::symbolic::ChebyshevBasisElement::operator<
        struct /* operator_lt */ {
          // Source: drake/common/symbolic/chebyshev_basis_element.h
          const char* doc =
R"""(Compares two ChebyshevBasisElement in lexicographic order.)""";
        } operator_lt;
      } ChebyshevBasisElement;
      // Symbol: drake::symbolic::ChebyshevPolynomial
      struct /* ChebyshevPolynomial */ {
        // Source: drake/common/symbolic/chebyshev_polynomial.h
        const char* doc =
R"""(Represents the Chebyshev polynomial of the first kind Tₙ(x). One
definition of Chebyshev polynomial of the first kind is Tₙ(cos(θ)) =
cos(nθ) It can also be defined recursively as

T₀(x) = 1 T₁(x) = x Tₙ₊₁(x) = 2xTₙ(x) − Tₙ₋₁(x))""";
        // Symbol: drake::symbolic::ChebyshevPolynomial::ChebyshevPolynomial
        struct /* ctor */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Constructs a Chebyshev polynomial Tₙ(x)

Parameter ``var``:
    The variable x

Parameter ``degree``:
    The Chebyshev polynomial is of degree n.

Precondition:
    degree >= 0.)""";
        } ctor;
        // Symbol: drake::symbolic::ChebyshevPolynomial::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Computes the differentiation of a Chebyshev polynomial dTₙ(x)/dx =
nUₙ₋₁(x) where Uₙ₋₁(x) is a Chebyshev polynomial of the second kind.
Uₙ₋₁(x) can be written as a summation of Chebyshev polynomials of the
first kind with lower degrees. - If n is even dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x),
j is odd and j <= n-1 - If n is odd dTₙ(x)/dx = 2n ∑ⱼ Tⱼ(x) - n, j is
even and j <= n-1 - A special case is that dT₀(x)/dx = 0.

Returns ``chebyshev_coeff_pairs``:
    . sum(chebyshev_coeff_pairs[j].first *
    chebyshev_coeff_pairs[j].second) is the differentiation dTₙ(x)/dx.
    If n is even, then chebyshev_coeff_pairs[j] = (T₂ⱼ₋₁(x), 2n). If n
    is odd, then chebyshev_coeff_pairs[j] = (T₂ⱼ(x), 2n) for j >= 1,
    and chebyshev_coeff_pairs[0] = (T₀(x), n). For the special case
    when degree() == 0, we return an empty vector.)""";
        } Differentiate;
        // Symbol: drake::symbolic::ChebyshevPolynomial::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Evaluates this Chebyshev polynomial at ``var_val``.)""";
        } Evaluate;
        // Symbol: drake::symbolic::ChebyshevPolynomial::ToPolynomial
        struct /* ToPolynomial */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Converts this Chebyshev polynomial to a polynomial with monomial
basis.)""";
        } ToPolynomial;
        // Symbol: drake::symbolic::ChebyshevPolynomial::degree
        struct /* degree */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Getter for the degree of the Chebyshev polynomial.)""";
        } degree;
        // Symbol: drake::symbolic::ChebyshevPolynomial::operator!=
        struct /* operator_ne */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Checks if this and ``other`` do not represent the same Chebyshev
polynomial.)""";
        } operator_ne;
        // Symbol: drake::symbolic::ChebyshevPolynomial::operator<
        struct /* operator_lt */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc =
R"""(Compare this to another Chebyshev polynomial, returns True if this is
regarded as less than the other, otherwise returns false.

If this.var() < other.var(), return True. If this.var() > other.var(),
return False. If this.var() == other.var(), then return this.degree()
< other.degree().

A special case is when this.degree() == 0 or other.degree() == 0. In
this case the variable doesn't matter, and we return this.degree() <
other.degree().)""";
        } operator_lt;
        // Symbol: drake::symbolic::ChebyshevPolynomial::var
        struct /* var */ {
          // Source: drake/common/symbolic/chebyshev_polynomial.h
          const char* doc = R"""(Getter for the variable.)""";
        } var;
      } ChebyshevPolynomial;
      // Symbol: drake::symbolic::CodeGen
      struct /* CodeGen */ {
        // Source: drake/common/symbolic/codegen.h
        const char* doc_3args_function_name_parameters_e =
R"""(For a given symbolic expression ``e``, generates two C functions,
``<function_name>`` and ``<function_name>_meta``. The generated
``<function_name>`` function takes an array of doubles for parameters
and returns an evaluation result. ``<function_name>_meta`` returns a
nested struct from which a caller can obtain the following
information: - ``.p.size``: the size of input parameters.

Parameter ``function_name``:
    Name of the generated C function.

Parameter ``parameters``:
    Vector of variables provide the ordering of symbolic variables.

Parameter ``e``:
    Symbolic expression to codegen.

For example, ``Codegen("f", {x, y}, 1 + sin(x) + cos(y))`` generates
the following string.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    double f(const double* p) {
        return (1 + sin(p[0]) + cos(p[1]));
    }
    typedef struct {
        /* p: input, vector 
        struct { int size; } p;
    } f_meta_t;
    f_meta_t f_meta() { return {{2}}; }

.. raw:: html

    </details>

Note that in this example ``x`` and ``y`` are mapped to ``p[0]`` and
``p[1]`` respectively because we passed ``{x, y}`` to ``Codegen``.)""";
        // Source: drake/common/symbolic/codegen.h
        const char* doc_3args_conststdstring_conststdvector_constEigenPlainObjectBase =
R"""(For a given symbolic dense matrix ``M``, generates two C functions,
``<function_name>`` and ``<function_name>_meta``. The generated
``<function_name>`` takes two parameters:

- const double* p : An array of doubles for input parameters. -
double* m : An array of doubles to store the evaluation result.

``<function_name>_meta()`` returns a nested struct from which a caller
can obtain the following information: - ``.p.size``: the size of input
parameters. - ``.m.rows``: the number of rows in the matrix. -
``.m.cols``: the number of columns in the matrix.

Please consider the following example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Eigen∷Matrix<symbolic∷Expression, 2, 2, Eigen∷ColMajor> M;
    M(0, 0) = 1.0;
    M(1, 0) = 3 + x + y;
    M(0, 1) = 4 * y;
    M(1, 1) = sin(x);
    CodeGen("f", {x, y}, M);

.. raw:: html

    </details>

When executed, the last line of the above example generates the
following code:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void f(const double* p, double* m) {
      m[0] = 1.000000;
      m[1] = (3 + p[0] + p[1]);
      m[2] = (4 * p[1]);
      m[3] = sin(p[0]);
    }
    typedef struct {
      /* p: input, vector 
      struct {
        int size;
      } p;
      /* m: output, matrix 
      struct {
        int rows;
        int cols;
      } m;
    } f_meta_t;
    f_meta_t f_meta() { return {{2}, {2, 2}}; }

.. raw:: html

    </details>

Note that in this example, the matrix ``M`` is stored in column-major
order and the ``CodeGen`` function respects the storage order in the
generated code. If ``M`` were stored in row-major order, ``CodeGen``
would return the following:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void f(const double* p, double* m) {
        m[0] = 1.000000;
        m[1] = (4 * p[1]);
        m[2] = (3 + p[0] + p[1]);
        m[3] = sin(p[0]);
    }

.. raw:: html

    </details>)""";
        // Source: drake/common/symbolic/codegen.h
        const char* doc_3args_function_name_parameters_M =
R"""(Please consider the following example which generates code for a 3x6
sparse matrix.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Eigen∷SparseMatrix<Expression, Eigen∷ColMajor> m(3, 6);
    m.insert(0, 0) = x;
    m.insert(0, 4) = z;
    m.insert(1, 2) = y;
    m.insert(2, 3) = y;
    m.insert(2, 5) = y;
    m.makeCompressed();
    // | x  0  0  0  z  0|
    // | 0  0  y  0  0  0|
    // | 0  0  0  y  0  y|
    CodeGen("f", {x, y, z}, m);

.. raw:: html

    </details>

When executed, the last line of the above example generates the
following code:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void f(const double* p,
           int* outer_indices,
           int* inner_indices,
           double* values) {
        outer_indices[0] = 0;
        outer_indices[1] = 1;
        outer_indices[2] = 1;
        outer_indices[3] = 2;
        outer_indices[4] = 3;
        outer_indices[5] = 4;
        outer_indices[6] = 5;
    
        inner_indices[0] = 0;
        inner_indices[1] = 1;
        inner_indices[2] = 2;
        inner_indices[3] = 0;
        inner_indices[4] = 2;
    
        values[0] = p[0];
        values[1] = p[1];
        values[2] = p[1];
        values[3] = p[2];
        values[4] = p[1];
    }
    
    typedef struct {
        /* p: input, vector 
        struct { int size; } p;
        /* m: output, matrix 
        struct {
            int rows;
            int cols;
            int non_zeros;
        } m;
    } f_meta_t;
    f_meta_t f_meta() { return {{3}, {3, 6, 5}}; }

.. raw:: html

    </details>

In the following example, we show how to use the generated function to
evaluate the symbolic matrix and construct a sparse matrix of double
using ``Eigen∷Map``.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // set up param, outer_indices, inner_indices, and values.
    f_meta_t meta = f_meta();
    const Eigen∷Vector3d param{1 /* x */, 2 /* y */, 3 /* z */};
    std∷vector<int> outer_indices(meta.m.cols + 1);
    std∷vector<int> inner_indices(meta.m.non_zeros);
    std∷vector<double> values(meta.m.non_zeros);
    
    // call f to fill outer_indices, inner_indices, and values.
    f(param.data(), outer_indices.data(), inner_indices.data(), values.data());
    
    // use Eigen∷Map to turn (outer_indices, inner_indices, values) into a
    // sparse matrix.
    Eigen∷Map<Eigen∷SparseMatrix<double, Eigen∷ColMajor>> map_sp(
        meta.m.rows, meta.m.cols, meta.m.non_zeros, outer_indices.data(),
        inner_indices.data(), values.data());
    const Eigen∷SparseMatrix<double> m_double{map_sp.eval()};

.. raw:: html

    </details>)""";
      } CodeGen;
      // Symbol: drake::symbolic::CodeGenVisitor
      struct /* CodeGenVisitor */ {
        // Source: drake/common/symbolic/codegen.h
        const char* doc = R"""(Visitor class for code generation.)""";
        // Symbol: drake::symbolic::CodeGenVisitor::CodeGen
        struct /* CodeGen */ {
          // Source: drake/common/symbolic/codegen.h
          const char* doc =
R"""(Generates C expression for the expression ``e``.)""";
        } CodeGen;
        // Symbol: drake::symbolic::CodeGenVisitor::CodeGenVisitor
        struct /* ctor */ {
          // Source: drake/common/symbolic/codegen.h
          const char* doc =
R"""(Constructs an instance of this visitor class using the vector of
variables, ``parameters``. This visitor will map a symbolic variable
``var`` into ``p[n]`` where ``n`` is the index of the variable ``var``
in the given ``parameters``.)""";
        } ctor;
        // Symbol: drake::symbolic::CodeGenVisitor::IdToIndexMap
        struct /* IdToIndexMap */ {
          // Source: drake/common/symbolic/codegen.h
          const char* doc = R"""()""";
        } IdToIndexMap;
      } CodeGenVisitor;
      // Symbol: drake::symbolic::ComputePolynomialBasisUpToDegree
      struct /* ComputePolynomialBasisUpToDegree */ {
        // Source: drake/common/symbolic/polynomial_basis.h
        const char* doc =
R"""(Returns all polynomial basis elements up to a given degree under the
graded reverse lexicographic order.

Template parameter ``rows``:
    Number of rows or Eigen∷Dynamic.

Template parameter ``BasisElement``:
    A derived class of PolynomialBasisElement.

Parameter ``vars``:
    The variables appearing in the polynomial basis.

Parameter ``degree``:
    The highest total degree of the polynomial basis elements.

Parameter ``degree_type``:
    If degree_type is kAny, then the polynomial basis elements'
    degrees are no larger than ``degree``. If degree_type is kEven,
    then the elements' degrees are even numbers no larger than
    ``degree``. If degree_type is kOdd, then the elements' degrees are
    odd numbers no larger than ``degree``. TODO(hongkai.dai): this
    will replace ComputeMonomialBasis in monomial_util.h.)""";
      } ComputePolynomialBasisUpToDegree;
      // Symbol: drake::symbolic::DecomposeAffineExpression
      struct /* DecomposeAffineExpression */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc =
R"""(Decomposes an affine combination ``e`` = c0 + c1 * v1 + ... cn * vn
into the following:

constant term : c0 coefficient vector : [c1, ..., cn] variable vector
: [v1, ..., vn]

Then, it extracts the coefficient and the constant term. A map from
variable ID to int, ``map_var_to_index``, is used to decide a
variable's index in a linear combination.

Precondition:
1. ``coeffs`` is a row vector of double, whose length matches with the size of
``map_var_to_index``.
2. e.is_polynomial() is true.
3. e is an affine expression.
4. all values in ``map_var_to_index`` should be in the range [0,
map_var_to_index.size())

    $Parameter ``e``:

The symbolic affine expression

Parameter ``map_var_to_index``:
    A mapping from variable ID to variable index, such that
    map_var_to_index[vi.get_ID()] = i.

Parameter ``coeffs``:
    A row vector. coeffs(i) = ci.

Parameter ``constant_term``:
    c0 in the equation above.

Returns:
    num_variable. Number of variables in the expression. 2 * x(0) + 3
    has 1 variable; 2 * x(0) + 3 * x(1) - 2 * x(0) has 1 variable,
    since the x(0) term cancels.

Raises:
    RuntimeError if the input expression is not affine.)""";
      } DecomposeAffineExpression;
      // Symbol: drake::symbolic::DecomposeAffineExpressions
      struct /* DecomposeAffineExpressions */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc_4args_expressions_vars_M_v =
R"""(Decomposes ``expressions`` into ``M`` * ``vars`` + ``v``.

Raises:
    RuntimeError if ``expressions`` is not affine in ``vars``.

Precondition:
    M.rows() == expressions.rows() && M.cols() == vars.rows().

Precondition:
    v.rows() == expressions.rows().)""";
        // Source: drake/common/symbolic/decompose.h
        const char* doc_4args_v_A_b_vars =
R"""(Given a vector of affine expressions v, decompose it to :math:`v = A
vars + b`

Parameter ``v``:
    A vector of affine expressions

Parameter ``A``:
    The matrix containing the linear coefficients.

Parameter ``b``:
    The vector containing all the constant terms.

Parameter ``vars``:
    All variables.

Raises:
    RuntimeError if the input expressions are not affine.)""";
      } DecomposeAffineExpressions;
      // Symbol: drake::symbolic::DecomposeL2NormExpression
      struct /* DecomposeL2NormExpression */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc =
R"""(Decomposes an L2 norm ``e`` = |Ax+b|₂ into A, b, and the variable
vector x (or returns false if the decomposition is not possible).

In order for the decomposition to succeed, the following conditions
must be met: 1. e is a sqrt expression. 2. e.get_argument() is a
polynomial of degree 2, which can be expressed as a quadratic form
(Ax+b)ᵀ(Ax+b).

Parameter ``e``:
    The symbolic affine expression

Parameter ``psd_tol``:
    The tolerance for checking positive semidefiniteness. Eigenvalues
    less that this threshold are considered to be zero. Matrices with
    negative eigenvalues less than this threshold are considered to be
    not positive semidefinite, and will cause the decomposition to
    fail.

Parameter ``coefficient_tol``:
    The absolute tolerance for checking that the coefficients of the
    expression inside the sqrt match the coefficients of |Ax+b|₂².

Returns:
    [is_l2norm, A, b, vars] where is_l2norm is true iff the
    decomposition was successful, and if is_l2norm is true then
    |A*vars + b|₂ = e.)""";
      } DecomposeL2NormExpression;
      // Symbol: drake::symbolic::DecomposeLinearExpressions
      struct /* DecomposeLinearExpressions */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc =
R"""(Decomposes ``expressions`` into ``M`` * ``vars``.

Raises:
    RuntimeError if ``expressions`` is not linear in ``vars``.

Precondition:
    M.rows() == expressions.rows() && M.cols() == vars.rows().)""";
      } DecomposeLinearExpressions;
      // Symbol: drake::symbolic::DecomposeLumpedParameters
      struct /* DecomposeLumpedParameters */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc =
R"""(Given a vector of Expressions ``f`` and a list of ``parameters`` we
define all additional variables in ``f`` to be a vector of
"non-parameter variables", n. This method returns a factorization of
``f`` into an equivalent "data matrix", W, which depends only on the
non-parameter variables, and a "lumped parameter vector", α, which
depends only on ``parameters:`` f = W(n)*α(parameters) + w0(n).

Note:
    The current implementation makes some simple attempts to minimize
    the number of lumped parameters, but more simplification could be
    implemented relatively easily. Optimal simplification, however,
    involves the complexity of comparing two arbitrary Expressions
    (see Expression∷EqualTo for more details).

Raises:
    RuntimeError if ``f`` is not decomposable in this way (cells
    containing ``parameters`` may only be added or multiplied with
    cells containing non-parameter variables).

Returns:
    W(n), α(parameters), and w0(n).)""";
      } DecomposeLumpedParameters;
      // Symbol: drake::symbolic::DecomposeQuadraticPolynomial
      struct /* DecomposeQuadraticPolynomial */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc =
R"""(Given a quadratic polynomial ``poly``, decomposes it into the form 0.5
* x' Q * x + b' * x + c

Parameter ``poly``:
    Quadratic polynomial to decompose.

Parameter ``map_var_to_index``:
    maps variables in ``poly.GetVariables()`` to the index in the
    vector ``x``.

Parameter ``Q``:
    The Hessian of the quadratic expression.

Precondition:
    The size of Q should be ``num_variables x num_variables``. Q is a
    symmetric matrix.

Parameter ``b``:
    linear term of the quadratic expression.

Precondition:
    The size of ``b`` should be ``num_variables``.

Parameter ``c``:
    The constant term of the quadratic expression.)""";
      } DecomposeQuadraticPolynomial;
      // Symbol: drake::symbolic::Evaluate
      struct /* Evaluate */ {
        // Source: drake/common/symbolic/polynomial.h
        const char* doc_polynomial =
R"""(Evaluates a matrix ``m`` of symbolic polynomials using ``env``.

Returns:
    a matrix of double whose size is the size of ``m``.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
      } Evaluate;
      // Symbol: drake::symbolic::EvaluateChebyshevPolynomial
      struct /* EvaluateChebyshevPolynomial */ {
        // Source: drake/common/symbolic/chebyshev_polynomial.h
        const char* doc =
R"""(Evaluates a Chebyshev polynomial at a given value.

Parameter ``var_val``:
    The value of the variable.

Parameter ``degree``:
    The degree of the Chebyshev polynomial.)""";
      } EvaluateChebyshevPolynomial;
      // Symbol: drake::symbolic::EvenDegreeMonomialBasis
      struct /* EvenDegreeMonomialBasis */ {
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc =
R"""(Returns all even degree monomials up to a given degree under the
graded reverse lexicographic order. A monomial has an even degree if
its total degree is even. So xy is an even degree monomial (degree 2)
while x²y is not (degree 3). Note that graded reverse lexicographic
order uses the total order among Variable which is based on a
variable's unique ID. For example, for a given variable ordering x > y
> z, ``EvenDegreeMonomialBasis({x, y, z}, 2)`` returns a column vector
``[x², xy, y², xz, yz, z², 1]``.

Precondition:
    ``vars`` is a non-empty set.

Precondition:
    ``degree`` is a non-negative integer.)""";
      } EvenDegreeMonomialBasis;
      // Symbol: drake::symbolic::ExtractAndAppendVariablesFromExpression
      struct /* ExtractAndAppendVariablesFromExpression */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc =
R"""(Given an expression ``e``, extracts all variables inside ``e``,
appends these variables to ``vars`` if they are not included in
``vars`` yet.

Parameter ``e``:
    A symbolic expression.

Parameter ``vars``:
    As an input, ``vars`` contain the variables before extracting
    expression ``e``. As an output, the variables in ``e`` that were
    not included in ``vars``, will be appended to the end of ``vars``.

Parameter ``map_var_to_index``:
    is of the same size as ``vars``, and
    map_var_to_index[vars(i).get_id()] = i. This invariance holds for
    map_var_to_index both as the input and as the output.)""";
      } ExtractAndAppendVariablesFromExpression;
      // Symbol: drake::symbolic::ExtractVariablesFromExpression
      struct /* ExtractVariablesFromExpression */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc_1args_e =
R"""(Given an expression ``e``, extracts all variables inside ``e``.

Parameter ``e``:
    A symbolic expression.

Returns ``pair``:
    pair.first is the variables in ``e``. pair.second is the mapping
    from the variable ID to the index in pair.first, such that
    pair.second[pair.first(i).get_id()] = i)""";
        // Source: drake/common/symbolic/decompose.h
        const char* doc_1args_expressions =
R"""(Overloads ExtractVariablesFromExpression but with a vector of
expressions.)""";
      } ExtractVariablesFromExpression;
      // Symbol: drake::symbolic::GenericPolynomial
      struct /* GenericPolynomial */ {
        // Source: drake/common/symbolic/generic_polynomial.h
        const char* doc =
R"""(Represents symbolic generic polynomials using a given basis (for
example, monomial basis, Chebyshev basis, etc). A generic symbolic
polynomial keeps a mapping from a basis element of indeterminates to
its coefficient in a symbolic expression. A generic polynomial ``p``
has to satisfy an invariant such that ``p.decision_variables() ∩
p.indeterminates() = ∅``. We have CheckInvariant() method to check the
invariant. For polynomials using different basis, you could refer to
section 3.1.5 of Semidefinite Optimization and Convex Algebraic
Geometry on the pros/cons of each basis.

We provide two instantiations of this template - BasisElement =
MonomialBasisElement - BasisElement = ChebyshevBasisElement

Template parameter ``BasisElement``:
    Must be a subclass of PolynomialBasisElement.)""";
        // Symbol: drake::symbolic::GenericPolynomial::AddProduct
        struct /* AddProduct */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Adds ``coeff`` * ``m`` to this generic polynomial.)""";
        } AddProduct;
        // Symbol: drake::symbolic::GenericPolynomial::CoefficientsAlmostEqual
        struct /* CoefficientsAlmostEqual */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns true if this polynomial and ``p`` are almost equal (the
difference in the corresponding coefficients are all less than
``tol)``, after expanding the coefficients.)""";
        } CoefficientsAlmostEqual;
        // Symbol: drake::symbolic::GenericPolynomial::Degree
        struct /* Degree */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns the highest degree of this generic polynomial in an
indeterminate ``v``.)""";
        } Degree;
        // Symbol: drake::symbolic::GenericPolynomial::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Differentiates this generic polynomial with respect to the variable
``x``. Note that a variable ``x`` can be either a decision variable or
an indeterminate.)""";
        } Differentiate;
        // Symbol: drake::symbolic::GenericPolynomial::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns true if this and ``p`` are structurally equal.)""";
        } EqualTo;
        // Symbol: drake::symbolic::GenericPolynomial::EqualToAfterExpansion
        struct /* EqualToAfterExpansion */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns true if this generic polynomial and ``p`` are equal after
expanding the coefficients.)""";
        } EqualToAfterExpansion;
        // Symbol: drake::symbolic::GenericPolynomial::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Evaluates this generic polynomial under a given environment ``env``.

Raises:
    RuntimeError if there is a variable in this generic polynomial
    whose assignment is not provided by ``env``.)""";
        } Evaluate;
        // Symbol: drake::symbolic::GenericPolynomial::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_1args =
R"""(Partially evaluates this generic polynomial using an environment
``env``.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_2args =
R"""(Partially evaluates this generic polynomial by substituting ``var``
with ``c``.

Raises:
    RuntimeError if NaN is detected at any point during evaluation.)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::GenericPolynomial::GenericPolynomial<BasisElement>
        struct /* ctor */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_0args = R"""(Constructs a zero polynomial.)""";
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_1args_stdnullptrt =
R"""(Constructs a default value. This overload is used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_1args_init =
R"""(Constructs a generic polynomial from a map, basis_element →
coefficient. For example


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cc}
    GenericPolynomial<MonomialBasiElement>(
      {{MonomialBasisElement(x, 2), a}, {MonomialBasisElement(x, 3), a+b}})

.. raw:: html

    </details>

constructs a polynomial ax²+(a+b)x³.)""";
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_1args_m =
R"""(Constructs a generic polynomial from a single basis element ``m``.

Note:
    that all variables in ``m`` are considered as indeterminates.
    Namely the constructed generic polynomial contains the map with a
    single key ``m``, with the coefficient being 1.)""";
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_1args_e =
R"""(Constructs a polynomial from an expression ``e``. Note that all
variables in ``e`` are considered as indeterminates.

Raises:
    RuntimeError if ``e`` is not a polynomial.)""";
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc_2args_e_indeterminates =
R"""(Constructs a polynomial from an expression ``e`` by decomposing it
with respect to ``indeterminates``.

Note:
    The indeterminates for the polynomial are ``indeterminates``. Even
    if a variable in ``indeterminates`` does not show up in ``e``,
    that variable is still registered as an indeterminate in this
    polynomial, as this->indeterminates() be the same as
    ``indeterminates``.

Raises:
    RuntimeError if ``e`` is not a polynomial in ``indeterminates``.)""";
        } ctor;
        // Symbol: drake::symbolic::GenericPolynomial::Jacobian
        struct /* Jacobian */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Computes the Jacobian matrix J of the generic polynomial with respect
to ``vars``. J(0,i) contains ∂f/∂vars(i). ``vars`` should be an Eigen
column vector of symbolic variables.)""";
        } Jacobian;
        // Symbol: drake::symbolic::GenericPolynomial::MapType
        struct /* MapType */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Type of mapping from basis element to coefficient)""";
        } MapType;
        // Symbol: drake::symbolic::GenericPolynomial::RemoveTermsWithSmallCoefficients
        struct /* RemoveTermsWithSmallCoefficients */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Removes the terms whose absolute value of the coefficients are smaller
than or equal to ``coefficient_tol``. For example, if the generic
polynomial is 2x² + 3xy + 10⁻⁴x - 10⁻⁵, then after calling
RemoveTermsWithSmallCoefficients(1e-3), the returned polynomial
becomes 2x² + 3xy.

Parameter ``coefficient_tol``:
    A positive scalar.

Returns ``polynomial_cleaned``:
    A generic polynomial whose terms with small coefficients are
    removed.)""";
        } RemoveTermsWithSmallCoefficients;
        // Symbol: drake::symbolic::GenericPolynomial::SetIndeterminates
        struct /* SetIndeterminates */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Sets the indeterminates to ``new_indeterminates``.

Changing the indeterminates will change
``basis_element_to_coefficient_map()``, and also potentially the
degree of the polynomial. Here is an example.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // p is a quadratic polynomial with x being the only indeterminate.
    symbolic∷GenericPolynomial<MonomialBasisElement> p(a * x * x + b * x + c,
    {x});
    // p.basis_element_to_coefficient_map() contains {1: c, x: b, x*x:a}.
    std∷cout << p.TotalDegree(); // prints 2.
    // Now set (a, b, c) to the indeterminates. p becomes a linear
    // polynomial of a, b, c.
    p.SetIndeterminates({a, b, c});
    // p.basis_element_to_coefficient_map() now is {a: x * x, b: x, c: 1}.
    std∷cout << p.TotalDegree(); // prints 1.

.. raw:: html

    </details>

This function can be expensive, as it potentially reconstructs the
polynomial (using the new indeterminates) from the expression.)""";
        } SetIndeterminates;
        // Symbol: drake::symbolic::GenericPolynomial::ToExpression
        struct /* ToExpression */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns an equivalent symbolic expression of this generic polynomial.)""";
        } ToExpression;
        // Symbol: drake::symbolic::GenericPolynomial::TotalDegree
        struct /* TotalDegree */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns the total degree of this generic polynomial.)""";
        } TotalDegree;
        // Symbol: drake::symbolic::GenericPolynomial::basis_element_to_coefficient_map
        struct /* basis_element_to_coefficient_map */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns the map from each basis element to its coefficient.)""";
        } basis_element_to_coefficient_map;
        // Symbol: drake::symbolic::GenericPolynomial::decision_variables
        struct /* decision_variables */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns the decision variables of this generic polynomial.)""";
        } decision_variables;
        // Symbol: drake::symbolic::GenericPolynomial::indeterminates
        struct /* indeterminates */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns the indeterminates of this generic polynomial.)""";
        } indeterminates;
        // Symbol: drake::symbolic::GenericPolynomial::operator!=
        struct /* operator_ne */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc =
R"""(Returns a symbolic formula representing the condition where this
polynomial and ``p`` are not the same.)""";
        } operator_ne;
        // Symbol: drake::symbolic::GenericPolynomial::operator*=
        struct /* operator_imul */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc = R"""()""";
        } operator_imul;
        // Symbol: drake::symbolic::GenericPolynomial::operator+=
        struct /* operator_iadd */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc = R"""()""";
        } operator_iadd;
        // Symbol: drake::symbolic::GenericPolynomial::operator-=
        struct /* operator_isub */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc = R"""()""";
        } operator_isub;
        // Symbol: drake::symbolic::GenericPolynomial::operator/=
        struct /* operator_idiv */ {
          // Source: drake/common/symbolic/generic_polynomial.h
          const char* doc = R"""()""";
        } operator_idiv;
      } GenericPolynomial;
      // Symbol: drake::symbolic::GradedReverseLexOrder
      struct /* GradedReverseLexOrder */ {
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc =
R"""(Implements Graded reverse lexicographic order.

Template parameter ``VariableOrder``:
    VariableOrder{}(v1, v2) is true if v1 < v2.

We first compare the total degree of the monomial; if there is a tie,
then we use the lexicographical order as the tie breaker, but a
monomial with higher order in lexicographical order is considered
lower order in graded reverse lexicographical order.

Take MonomialBasis({x, y, z}, 2) as an example, with the order x > y >
z. To get the graded reverse lexicographical order, we take the
following steps:

First find all the monomials using the total degree. The monomials
with degree 2 are {x^2, y^2, z^2, xy, xz, yz}. The monomials with
degree 1 are {x, y, z}, and the monomials with degree 0 is {1}. To
break the tie between monomials with the same total degree, first sort
them in the reverse lexicographical order, namely x < y < z in the
reverse lexicographical order. The lexicographical order compares two
monomial by first comparing the exponent of the largest variable, if
there is a tie then go forth to the second largest variable. Thus z^2
> zy >zx > y^2 > yx > x^2. Finally reverse the order as x^2 > xy > y^2
> xz > yz > z^2.

There is an introduction to monomial order in
https://en.wikipedia.org/wiki/Monomial_order, and an introduction to
graded reverse lexicographical order in
https://en.wikipedia.org/wiki/Monomial_order#Graded_reverse_lexicographic_order)""";
        // Symbol: drake::symbolic::GradedReverseLexOrder::operator()
        struct /* operator_call */ {
          // Source: drake/common/symbolic/monomial_util.h
          const char* doc =
R"""(Returns true if m1 > m2 under the Graded reverse lexicographic order.)""";
        } operator_call;
      } GradedReverseLexOrder;
      // Symbol: drake::symbolic::IsAffine
      struct /* IsAffine */ {
        // Source: drake/common/symbolic/decompose.h
        const char* doc_2args =
R"""(Checks if every element in ``m`` is affine in ``vars``.

Note:
    If ``m`` is an empty matrix, it returns true.)""";
        // Source: drake/common/symbolic/decompose.h
        const char* doc_1args =
R"""(Checks if every element in ``m`` is affine.

Note:
    If ``m`` is an empty matrix, it returns true.)""";
      } IsAffine;
      // Symbol: drake::symbolic::Jacobian
      struct /* Jacobian */ {
        // Source: drake/common/symbolic/polynomial.h
        const char* doc_polynomial =
R"""(Computes the Jacobian matrix J of the vector function ``f`` with
respect to ``vars``. J(i,j) contains ∂f(i)/∂vars(j).

Precondition:
    ``vars`` is non-empty.)""";
      } Jacobian;
      // Symbol: drake::symbolic::MakeRuleRewriter
      struct /* MakeRuleRewriter */ {
        // Source: drake/common/symbolic/simplification.h
        const char* doc =
R"""(Constructs a rewriter based on a rewriting rule ``r``.)""";
      } MakeRuleRewriter;
      // Symbol: drake::symbolic::Monomial
      struct /* Monomial */ {
        // Source: drake/common/symbolic/monomial.h
        const char* doc =
R"""(Represents a monomial, a product of powers of variables with
non-negative integer exponents. Note that it does not include the
coefficient part of a monomial.)""";
        // Symbol: drake::symbolic::Monomial::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc_1args =
R"""(Evaluates under a given environment ``env``.

Raises:
    RuntimeError if there is a variable in this monomial whose
    assignment is not provided by ``env``.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_2args =
R"""(Evaluates the monomial for a batch of data. We return monomial_vals
such that monomial_vals(j) is obtained by substituting ``vars(i)``
with ``vars_values(i, j)``, note that ``vars_values.rows() ==
vars.rows()`` and ``vars_values.cols() == monomial_vals.rows()``.

Parameter ``vars``:
    The variables whose value will be substituted. ``vars`` must
    contain all variables in this->GetVariables(). Also ``vars``
    cannot contain any duplicate variables, namely vars(i) != vars(j)
    if i != j.

Parameter ``vars_values``:
    The i'th column of ``vars_values`` is the i'th data for ``vars``.

Raises:
    RuntimeError if ``vars`` doesn't contain all the variables in
    ``this->GetVariables()``.)""";
        } Evaluate;
        // Symbol: drake::symbolic::Monomial::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Partially evaluates using a given environment ``env``. The evaluation
result is of type pair<double, Monomial>. The first component (a
double) represents the coefficient part while the second component
represents the remaining parts of the monomial which was not
evaluated.

Example 1. Evaluate with a fully-specified environment
(x³*y²).EvaluatePartial({{x, 2}, {y, 3}}) = (2³ * 3² = 8 * 9 = 72,
Monomial{} = 1).

Example 2. Evaluate with a partial environment
(x³*y²).EvaluatePartial({{x, 2}}) = (2³ = 8, y²).)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::Monomial::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns the set of variables in this monomial.)""";
        } GetVariables;
        // Symbol: drake::symbolic::Monomial::Monomial
        struct /* ctor */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc_0args =
R"""(Constructs a monomial equal to 1. Namely the total degree is zero.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_1args_stdnullptrt =
R"""(Constructs a default value. This overload is used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_1args_powers =
R"""(Constructs a monomial from ``powers``.

Raises:
    RuntimeError if ``powers`` includes a negative exponent.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_2args_vars_exponents =
R"""(Constructs a monomial from a vector of variables ``vars`` and their
corresponding integer exponents ``exponents``. For example,
``%Monomial([x, y, z], [2, 0, 1])`` constructs a monomial ``x²z``.

Precondition:
    The size of ``vars`` should be the same as the size of
    ``exponents``.

Raises:
    RuntimeError if ``exponents`` includes a negative integer.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_1args_e =
R"""(Converts an expression to a monomial if the expression is written as
∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.

Precondition:
    is_polynomial(e) should be true.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_1args_var = R"""(Constructs a monomial from ``var``.)""";
          // Source: drake/common/symbolic/monomial.h
          const char* doc_2args_var_exponent =
R"""(Constructs a monomial from ``var`` and ``exponent``.)""";
        } ctor;
        // Symbol: drake::symbolic::Monomial::ToExpression
        struct /* ToExpression */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns a symbolic expression representing this monomial.)""";
        } ToExpression;
        // Symbol: drake::symbolic::Monomial::degree
        struct /* degree */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns the degree of this monomial in a variable ``v``.)""";
        } degree;
        // Symbol: drake::symbolic::Monomial::get_powers
        struct /* get_powers */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns the internal representation of Monomial, the map from a base
(Variable) to its exponent (int).)""";
        } get_powers;
        // Symbol: drake::symbolic::Monomial::operator!=
        struct /* operator_ne */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Checks if this monomial and ``m`` do not represent the same monomial.)""";
        } operator_ne;
        // Symbol: drake::symbolic::Monomial::operator*=
        struct /* operator_imul */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns this monomial multiplied by ``m``.)""";
        } operator_imul;
        // Symbol: drake::symbolic::Monomial::pow_in_place
        struct /* pow_in_place */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns this monomial raised to ``p``.

Raises:
    RuntimeError if ``p`` is negative.)""";
        } pow_in_place;
        // Symbol: drake::symbolic::Monomial::to_string
        struct /* to_string */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns the string representation of this monomial.)""";
        } to_string;
        // Symbol: drake::symbolic::Monomial::total_degree
        struct /* total_degree */ {
          // Source: drake/common/symbolic/monomial.h
          const char* doc =
R"""(Returns the total degree of this monomial.)""";
        } total_degree;
      } Monomial;
      // Symbol: drake::symbolic::MonomialBasis
      struct /* MonomialBasis */ {
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc_2args_vars_degree =
R"""(Returns all monomials up to a given degree under the graded reverse
lexicographic order. Note that graded reverse lexicographic order uses
the total order among Variable which is based on a variable's unique
ID. For example, for a given variable ordering x > y > z,
``MonomialBasis({x, y, z}, 2)`` returns a column vector ``[x^2, xy,
y^2, xz, yz, z^2, x, y, z, 1]``.

Precondition:
    ``vars`` is a non-empty set.

Precondition:
    ``degree`` is a non-negative integer.)""";
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc_1args_constVariables =
R"""(Returns all monomials up to a given degree under the graded reverse
lexicographic order.

Template parameter ``n``:
    number of variables.

Template parameter ``degree``:
    maximum total degree of monomials to compute.

Precondition:
    ``vars`` is a non-empty set.

Precondition:
    vars.size() == ``n``.)""";
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc_1args_variables_degree =
R"""(Returns all the monomials (in graded reverse lexicographic order) such
that the total degree for each set of variables is no larger than a
specific degree. For example if x_set = {x₀, x₁} and y_set = {y₀, y₁},
then MonomialBasis({{x_set, 2}, {y_set, 1}}) will include all the
monomials, whose total degree of x_set is no larger than 2, and the
total degree of y_set is no larger than 1. Hence it can include
monomials such as x₀x₁y₀, but not x₀y₀y₁ because the total degree for
y_set is 2. So it would return the following set of monomials
(ignoring the ordering) {x₀²y₀, x₀²y₁, x₀x₁y₀, x₀x₁y₁, x₁²y₀, x₁²y₀,
x₀y₀, x₀y₁, x₁y₀, x₁y₁, x₀², x₀x₁, x₁², x₀, x₁, y₀, y₁, 1}.

Parameter ``variables_degree``:
    ``(vars, degree)`` maps each set of variables ``vars`` to the
    maximal degree of these variables in the monomial. Namely the
    summation of the degree of each variable in ``vars`` is no larger
    than ``degree``.

Precondition:
    The variables in ``variables_degree`` don't overlap.

Precondition:
    The degree in ``variables_degree`` are non-negative.)""";
      } MonomialBasis;
      // Symbol: drake::symbolic::MonomialBasisElement
      struct /* MonomialBasisElement */ {
        // Source: drake/common/symbolic/monomial_basis_element.h
        const char* doc =
R"""(MonomialBasisElement represents a monomial, a product of powers of
variables with non-negative integer exponents. Note that it doesn't
not include the coefficient part of a monomial. So x, x³y, xy²z are
all valid MonomialBasisElement instances, but 1+x or 2xy²z are not.
TODO(hongkai.dai): deprecate Monomial class and replace Monomial class
with MonomialBasisElement class. For more information regarding the
motivation of this class, please see Drake github issue #13602 and
#13803.)""";
        // Symbol: drake::symbolic::MonomialBasisElement::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Differentiates this MonomialBasisElement. Since dxⁿ/dx = nxⁿ⁻¹, we
return the map from the MonomialBasisElement to its coefficient. So if
this MonomialBasisElement is x³y², then differentiate with x will
return (x²y² → 3) as dx³y²/dx = 3x²y² If ``var`` is not a variable in
MonomialBasisElement, then returns an empty map.)""";
        } Differentiate;
        // Symbol: drake::symbolic::MonomialBasisElement::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Partially evaluates using a given environment ``env``. The evaluation
result is of type pair<double, MonomialBasisElement>. The first
component (: double) represents the coefficient part while the second
component represents the remaining parts of the MonomialBasisElement
which was not evaluated.

Example 1. Evaluate with a fully-specified environment
(x³*y²).EvaluatePartial({{x, 2}, {y, 3}}) = (2³ * 3² = 8 * 9 = 72,
MonomialBasisElement{} = 1).

Example 2. Evaluate with a partial environment
(x³*y²).EvaluatePartial({{x, 2}}) = (2³ = 8, y²).)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::MonomialBasisElement::Integrate
        struct /* Integrate */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Integrates this MonomialBasisElement on a variable. Since ∫ xⁿ dx = 1
/ (n+1) xⁿ⁺¹, we return the map from the MonomialBasisElement to its
coefficient in the integration result. So if this MonomialBasisElement
is x³y², then we return (x⁴y² → 1/4) as ∫ x³y²dx = 1/4 x⁴y². If
``var`` is not a variable in this MonomialBasisElement, for example ∫
x³y²dz = x³y²z, then we return (x³y²z → 1))""";
        } Integrate;
        // Symbol: drake::symbolic::MonomialBasisElement::MergeBasisElementInPlace
        struct /* MergeBasisElementInPlace */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Merges this basis element with another basis element ``other`` by
merging their var_to_degree_map. This is equivalent to multiplying
this monomial basis element in place with monomial basis element
``other``.)""";
        } MergeBasisElementInPlace;
        // Symbol: drake::symbolic::MonomialBasisElement::MonomialBasisElement
        struct /* ctor */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_0args =
R"""(Constructs a monomial equal to 1. Namely the toal degree is zero.)""";
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_1args_stdnullptrt =
R"""(Constructs a default value. This overload is used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_1args_var_to_degree_map =
R"""(Constructs a MonomialBasisElement from variable to degree map.)""";
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_1args_e =
R"""(Converts an expression to a monomial if the expression is written as
∏ᵢpow(xᵢ, kᵢ), otherwise throws a runtime error.

Precondition:
    is_polynomial(e) should be true.)""";
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_2args_vars_degrees =
R"""(Constructs a Monomial from a vector of variables ``vars`` and their
corresponding integer degrees ``degrees``. For example,
``MonomialBasisElement([x, y, z], [2, 0, 1])`` constructs a
MonomialBasisElement ``x²z``.

Precondition:
    The size of ``vars`` should be the same as the size of
    ``degrees``.

Raises:
    RuntimeError if ``degrees`` includes a negative integer.)""";
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_1args_var =
R"""(Constructs a monomial basis element with only one variable, and the
degree is 1.)""";
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc_2args_var_degree =
R"""(Constructs a monomial basis element with only one variable, and the
degree of that variable is given by ``degree``.)""";
        } ctor;
        // Symbol: drake::symbolic::MonomialBasisElement::ToBasis
        struct /* ToBasis */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Converts this monomial to a weighted sum of basis elements of type
BasisElement. We return the map from each BasisElement to its
coefficient. For example, if BasisElement=ChebyshevBasisElement, then
when this = x²y³, it returns {[T₂(x)T₃(y)⇒1/8], [T₂(x)T₁(y)⇒3/8],
[T₀(x)T₃(y)⇒1/8], [T₀(x)T₁(y)⇒3/8]}.

Note:
    Currently we only support

Template parameter ``BasisElement``:
    being MonomialBasisElement and ChebyshevBasisElement.)""";
        } ToBasis;
        // Symbol: drake::symbolic::MonomialBasisElement::ToChebyshevBasis
        struct /* ToChebyshevBasis */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Converts this monomial to Chebyshev polynomial basis. For example,

- For x², it returns 0.5T₂(x) + 0.5T₀(x). - For x²y³, it returns
1/8T₂(x)T₃(y) + 3/8T₂(x)T₁(y) + 1/8T₀(x)T₃(y) + 3/8T₀(x)T₁(y).

We return the map from each ChebyshevBasisElement to its coefficient.
For example, when this = x², it returns {[T₂(x)⇒0.5], [T₀(x)⇒0.5]}.
When this = x²y³, it returns {[T₂(x)T₃(y)⇒1/8], [T₂(x)T₁(y)⇒3/8],
[T₀(x)T₃(y)⇒1/8], [T₀(x)T₁(y)⇒3/8]}.)""";
        } ToChebyshevBasis;
        // Symbol: drake::symbolic::MonomialBasisElement::operator<
        struct /* operator_lt */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Compares two MonomialBasisElement in lexicographic order.)""";
        } operator_lt;
        // Symbol: drake::symbolic::MonomialBasisElement::pow_in_place
        struct /* pow_in_place */ {
          // Source: drake/common/symbolic/monomial_basis_element.h
          const char* doc =
R"""(Returns this monomial raised to ``p``.

Raises:
    RuntimeError if ``p`` is negative.)""";
        } pow_in_place;
      } MonomialBasisElement;
      // Symbol: drake::symbolic::NChooseK
      struct /* NChooseK */ {
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc = R"""()""";
      } NChooseK;
      // Symbol: drake::symbolic::OddDegreeMonomialBasis
      struct /* OddDegreeMonomialBasis */ {
        // Source: drake/common/symbolic/monomial_util.h
        const char* doc =
R"""(Returns all odd degree monomials up to a given degree under the graded
reverse lexicographic order. A monomial has an odd degree if its total
degree is odd. So x²y is an odd degree monomial (degree 3) while xy is
not (degree 2). Note that graded reverse lexicographic order uses the
total order among Variable which is based on a variable's unique ID.
For example, for a given variable ordering x > y > z,
``OddDegreeMonomialBasis({x, y, z}, 3)`` returns a column vector
``[x³, x²y, xy², y³, x²z, xyz, y²z, xz², yz², z³, x, y, z]``

Precondition:
    ``vars`` is a non-empty set.

Precondition:
    ``degree`` is a non-negative integer.)""";
      } OddDegreeMonomialBasis;
      // Symbol: drake::symbolic::Pattern
      struct /* Pattern */ {
        // Source: drake/common/symbolic/simplification.h
        const char* doc =
R"""(A pattern is an expression which possibly includes variables which
represent placeholders. It is used to construct a ``RewritingRule``.)""";
      } Pattern;
      // Symbol: drake::symbolic::Polynomial
      struct /* Polynomial */ {
        // Source: drake/common/symbolic/polynomial.h
        const char* doc =
R"""(Represents symbolic polynomials. A symbolic polynomial keeps a mapping
from a monomial of indeterminates to its coefficient in a symbolic
expression.

A polynomial ``p`` has to satisfy an invariant such that
``p.decision_variables() ∩ p.indeterminates() = ∅``. We have
CheckInvariant() method to check the invariant.

** Operation between a Polynomial and a Variable **

Note that for arithmetic operations ⊕ (where ⊕ can be +,-,*) between a
Polynomial ``p`` and a Variable ``v``, if the variable ``v`` is an
indeterminate of the polynomial ``p``, then we regard ``v`` as a
monomial with indeterminate ``v`` with coefficient 1; on the other
hand, if the variable ``v`` is not an indeterminate of ``p``, then we
regard ``v`` as a monomial with 0 degree and coefficient ``v``, and
``v`` will be appended to decision_variables() of the resulted
polynomial. For example, if p = ax²+y where (x,y) are indeterminates
and a is a decision variable, then in the operation p + y, the result
stores the monomial-to-coefficient mapping as {(x² -> a), (y -> 2)};
on the other hand, in the operation p + b, b is regarded as a 0-degree
monomial with coefficient b, and p + b stores the
monomial-to-coefficient mapping as {(x² -> a), (y -> 1), (1 -> b)},
with (p + b).decision_variables() being {a, b}. If you want to append
the variable ``v`` to the indeterminates of the p⊕v, then explicitly
convert it to a monomial as p ⊕ symbolic∷Monomial(v).)""";
        // Symbol: drake::symbolic::Polynomial::AddProduct
        struct /* AddProduct */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Adds ``coeff`` * ``m`` to this polynomial.)""";
        } AddProduct;
        // Symbol: drake::symbolic::Polynomial::CoefficientsAlmostEqual
        struct /* CoefficientsAlmostEqual */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns true if this polynomial and ``p`` are almost equal (the
difference in the corresponding coefficients are all less than
``tolerance``), after expanding the coefficients.)""";
        } CoefficientsAlmostEqual;
        // Symbol: drake::symbolic::Polynomial::Degree
        struct /* Degree */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns the highest degree of this polynomial in a variable ``v``.)""";
        } Degree;
        // Symbol: drake::symbolic::Polynomial::Differentiate
        struct /* Differentiate */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Differentiates this polynomial with respect to the variable ``x``.
Note that a variable ``x`` can be either a decision variable or an
indeterminate.)""";
        } Differentiate;
        // Symbol: drake::symbolic::Polynomial::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns true if this polynomial and ``p`` are structurally equal.)""";
        } EqualTo;
        // Symbol: drake::symbolic::Polynomial::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Evaluates this polynomial under a given environment ``env``.

Raises:
    RuntimeError if there is a variable in this polynomial whose
    assignment is not provided by ``env``.)""";
        } Evaluate;
        // Symbol: drake::symbolic::Polynomial::EvaluateIndeterminates
        struct /* EvaluateIndeterminates */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Evaluates the polynomial at a batch of indeterminates values.

Parameter ``indeterminates``:
    Must include all ``this->indeterminates()``

Parameter ``indeterminates_values``:
    Each column of ``indeterminates_values`` stores one specific value
    of ``indeterminates``; `indeterminates_values.rows() ==
    indeterminates.rows()`.

Returns:
    polynomial_values polynomial_values(j) is obtained by substituting
    indeterminates(i) in this polynomial with indeterminates_values(i,
    j) for all i.

Raises:
    RuntimeError if any coefficient in this polynomial is not a
    constant.)""";
        } EvaluateIndeterminates;
        // Symbol: drake::symbolic::Polynomial::EvaluatePartial
        struct /* EvaluatePartial */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args =
R"""(Partially evaluates this polynomial using an environment ``env``.

Raises:
    RuntimeError if NaN is detected during evaluation.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_2args =
R"""(Partially evaluates this polynomial by substituting ``var`` with
``c``.

Raises:
    RuntimeError if NaN is detected at any point during evaluation.)""";
        } EvaluatePartial;
        // Symbol: drake::symbolic::Polynomial::EvaluateWithAffineCoefficients
        struct /* EvaluateWithAffineCoefficients */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Evaluates the polynomial at a batch of indeterminate values. For a
polynomial whose coefficients are affine expressions of decision
variables, we evaluate this polynomial on a batch of indeterminate
values, and return the matrix representation of the evaluated affine
expressions. For example if p(x) = (a+1)x² + b*x where a, b are
decision variables, if we evaluate this polynomial on x = 1 and x = 2,
then p(x) = a+b+1 and 4a+2b+4 respectively. We return the evaluation
result as A * decision_variables + b, where A.row(i) *
decision_variables + b(i) is the evaluation of the polynomial on
indeterminates_values.col(i).

Parameter ``indeterminates``:
    Must include all this->indeterminates()

Parameter ``indeterminates_values``:
    A matrix representing a batch of values. Each column of
    ``indeterminates_values`` stores one specific value of
    ``indeterminates``, where ``indeterminates_values.rows() ==
    indeterminates.rows()``.

Parameter ``A``:
    The coefficient of the evaluation results.

Parameter ``decision_variables``:
    The decision variables in the evaluation results.

Parameter ``b``:
    The constant terms in the evaluation results.

Raises:
    RuntimeError if the coefficients of this polynomial is not an
    affine expression of its decision variables. For example, the
    polynomial (2+sin(a)) * x² + 1 (where ``a`` is a decision variable
    and ``x`` is a indeterminate) doesn't have affine expression as
    its coefficient 2+sin(a).)""";
        } EvaluateWithAffineCoefficients;
        // Symbol: drake::symbolic::Polynomial::Expand
        struct /* Expand */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Expands each coefficient expression and returns the expanded
polynomial. If any coefficient is equal to 0 after expansion, then
remove that term from the returned polynomial.)""";
        } Expand;
        // Symbol: drake::symbolic::Polynomial::Integrate
        struct /* Integrate */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args =
R"""(Integrates this polynomial with respect to an indeterminate ``x``.
Integration with respect to decision variables is not supported yet.
If ``x`` is not an indeterminate nor decision variable, then it will
be added to the list of indeterminates.

Raises:
    RuntimeError if ``x`` is a decision variable.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_3args =
R"""(Computes the definite integrate of this polynomial with respect to the
indeterminate ``x`` over the domain [a, b]. Integration with respect
to decision variables is not supported yet.

Raises:
    RuntimeError if ``x`` is a decision variable.)""";
        } Integrate;
        // Symbol: drake::symbolic::Polynomial::IsEven
        struct /* IsEven */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns true if the polynomial is even, namely p(x) = p(-x). Meaning
that the coefficient for all odd-degree monomials are 0. Returns false
otherwise. Note that this is different from the p.TotalDegree() being
an even number.)""";
        } IsEven;
        // Symbol: drake::symbolic::Polynomial::IsOdd
        struct /* IsOdd */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns true if the polynomial is odd, namely p(x) = -p(-x). Meaning
that the coefficient for all even-degree monomials are 0. Returns
false otherwise. Note that this is different from the p.TotalDegree()
being an odd number.)""";
        } IsOdd;
        // Symbol: drake::symbolic::Polynomial::Jacobian
        struct /* Jacobian */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Computes the Jacobian matrix J of the polynomial with respect to
``vars``. J(0,i) contains ∂f/∂vars(i).)""";
        } Jacobian;
        // Symbol: drake::symbolic::Polynomial::MapType
        struct /* MapType */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc = R"""()""";
        } MapType;
        // Symbol: drake::symbolic::Polynomial::Polynomial
        struct /* ctor */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_0args = R"""(Constructs a zero polynomial.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args_stdnullptrt =
R"""(Constructs a default value. This overload is used by Eigen when
EIGEN_INITIALIZE_MATRICES_BY_ZERO is enabled.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args_map =
R"""(Constructs a polynomial from a map, Monomial → Expression.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args_m =
R"""(Constructs a polynomial from a monomial ``m``. Note that all variables
in ``m`` are considered as indeterminates. Note that this implicit
conversion is desirable to have a dot product of two
Eigen∷Vector<Monomial>s return a Polynomial.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args_v =
R"""(Constructs a polynomial from a varaible ``v``. Note that v is
considered an indeterminate.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_1args_e =
R"""(Constructs a polynomial from an expression ``e``. Note that all
variables in ``e`` are considered as indeterminates.

Raises:
    RuntimeError if ``e`` is not a polynomial.)""";
          // Source: drake/common/symbolic/polynomial.h
          const char* doc_2args_e_indeterminates =
R"""(Constructs a polynomial from an expression ``e`` by decomposing it
with respect to ``indeterminates``.

Note:
    It collects the intersection of the variables appeared in ``e``
    and the provided ``indeterminates``.

Raises:
    RuntimeError if ``e`` is not a polynomial in ``indeterminates``.)""";
        } ctor;
        // Symbol: drake::symbolic::Polynomial::RemoveTermsWithSmallCoefficients
        struct /* RemoveTermsWithSmallCoefficients */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Removes the terms whose absolute value of the coefficients are smaller
than or equal to ``coefficient_tol``. For example, if the polynomial
is 2x² + 3xy + 10⁻⁴x - 10⁻⁵, then after calling
RemoveTermsWithSmallCoefficients(1e-3), the returned polynomial
becomes 2x² + 3xy.

Parameter ``coefficient_tol``:
    A positive scalar.

Returns ``polynomial_cleaned``:
    A polynomial whose terms with small coefficients are removed.)""";
        } RemoveTermsWithSmallCoefficients;
        // Symbol: drake::symbolic::Polynomial::Roots
        struct /* Roots */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns the roots of a *univariate* polynomial with constant
coefficients as a column vector. There is no specific guarantee on the
order of the returned roots.

Raises:
    RuntimeError if ``this`` is not univariate with constant
    coefficients.)""";
        } Roots;
        // Symbol: drake::symbolic::Polynomial::SetIndeterminates
        struct /* SetIndeterminates */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Sets the indeterminates to ``new_indeterminates``. Changing the
indeterminates would change ``monomial_to_coefficient_map()``, and
also potentially the degree of the polynomial. Here is an example.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // p is a quadratic polynomial with x being the indeterminates.
    symbolic∷Polynomial p(a * x * x + b * x + c, {x});
    // p.monomial_to_coefficient_map() contains {1: c, x: b, x*x:a}.
    std∷cout << p.TotalDegree(); // prints 2.
    // Now set (a, b, c) to the indeterminates. p becomes a linear
    // polynomial of a, b, c.
    p.SetIndeterminates({a, b, c});
    // p.monomial_to_coefficient_map() now is {a: x * x, b: x, c: 1}.
    std∷cout << p.TotalDegree(); // prints 1.

.. raw:: html

    </details>)""";
        } SetIndeterminates;
        // Symbol: drake::symbolic::Polynomial::SubstituteAndExpand
        struct /* SubstituteAndExpand */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Substitutes the monomials of this polynomial with new polynomial
expressions and expand the polynomial to the monomial basis. For
example, consider the substitution x = a(1-y) into the polynomial x¹⁴
+ (x−1)². Repeatedly expanding the powers of x can take a long time
using factory methods, so we store intermediate computations in the
substitution map to avoid recomputing very high powers.

Parameter ``indeterminate_substitution``:
    The substitutions of every indeterminate with the new desired
    expression. This map must contain each element of
    ``indeterminates()``. For performance reasons, it is recommended
    that this map contains Expanded polynomials as its values, but
    this is not necessary.

Parameter ``substitutions_cached_data``:
    A container caching the higher order expansions of the
    ``indeterminate_substitutions``. Typically, the first time an
    indeterminate_substitution is performed, this will be empty. If
    the same indeterminate_substitutions is used for multiple
    polynomials, passing this value will enable the user to re-use the
    expansions across multiple calls. For example, suppose we wish to
    perform the substitution x = a(1-y) into the polynomials p1 = x¹⁴
    + (x−1)² and p2 = x⁷. A user may call p1.SubstituteAndExpand({x :
    a(1-y), substitutions_cached_data}) where
    substitutions_cached_data is a pointer to an empty container. As
    part of computing the expansion of p1, the expansion of x⁷ may get
    computed and stored in substitutions_cached_data, and so a
    subsequent call of p2.SubstituteAndExpand({x : a(1-y)},
    substitutions_cached_data) would be very fast.

Never reuse substitutions_cached_data if indeterminate_substitutions
changes as this function will then compute an incorrect result.

Note that this function is NOT responsible for ensuring that
``substitutions_cached_data`` is consistent i.e. this method will not
throw an error if substitutions_cached_data contains the inconsistent
substitutions {x: y, x²: 2y}. To ensure correct results, ensure that
the passed substitutions_cached_data object is consistent with
indeterminate_substitutions. The easiest way to do this is to pass a
pointer to an empty substitutions_cached_data or nullopt to this
function.)""";
        } SubstituteAndExpand;
        // Symbol: drake::symbolic::Polynomial::SubstituteAndExpandCacheData
        struct /* SubstituteAndExpandCacheData */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(An encapsulated data type for use with the method SubstituteAndExpand.)""";
          // Symbol: drake::symbolic::Polynomial::SubstituteAndExpandCacheData::get_data
          struct /* get_data */ {
            // Source: drake/common/symbolic/polynomial.h
            const char* doc = R"""()""";
          } get_data;
        } SubstituteAndExpandCacheData;
        // Symbol: drake::symbolic::Polynomial::ToExpression
        struct /* ToExpression */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns an equivalent symbolic expression of this polynomial.)""";
        } ToExpression;
        // Symbol: drake::symbolic::Polynomial::TotalDegree
        struct /* TotalDegree */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns the total degree of this polynomial.)""";
        } TotalDegree;
        // Symbol: drake::symbolic::Polynomial::decision_variables
        struct /* decision_variables */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns the decision variables of this polynomial.)""";
        } decision_variables;
        // Symbol: drake::symbolic::Polynomial::indeterminates
        struct /* indeterminates */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns the indeterminates of this polynomial.)""";
        } indeterminates;
        // Symbol: drake::symbolic::Polynomial::monomial_to_coefficient_map
        struct /* monomial_to_coefficient_map */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns the mapping from a Monomial to its corresponding coefficient
of this polynomial. We maintain the invariance that for any [monomial,
coeff] pair in monomial_to_coefficient_map(), symbolic:is_zero(coeff)
is false.)""";
        } monomial_to_coefficient_map;
        // Symbol: drake::symbolic::Polynomial::operator!=
        struct /* operator_ne */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Returns a symbolic formula representing the condition where this
polynomial and ``p`` are not the same.)""";
        } operator_ne;
        // Symbol: drake::symbolic::Polynomial::operator*=
        struct /* operator_imul */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Depending on whether ``v`` is an indeterminate of this polynomial,
this operation generates different results. Refer to
polynomial_variable_operation "the class documentation" for more
details.)""";
        } operator_imul;
        // Symbol: drake::symbolic::Polynomial::operator+=
        struct /* operator_iadd */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Depending on whether ``v`` is an indeterminate of this polynomial,
this operation generates different results. Refer to
polynomial_variable_operation "the class documentation" for more
details.)""";
        } operator_iadd;
        // Symbol: drake::symbolic::Polynomial::operator-=
        struct /* operator_isub */ {
          // Source: drake/common/symbolic/polynomial.h
          const char* doc =
R"""(Depending on whether ``v`` is an indeterminate of this polynomial,
this operation generates different results. Refer to
polynomial_variable_operation "the class documentation" for more
details.)""";
        } operator_isub;
      } Polynomial;
      // Symbol: drake::symbolic::PolynomialBasisElement
      struct /* PolynomialBasisElement */ {
        // Source: drake/common/symbolic/polynomial_basis_element.h
        const char* doc =
R"""(Each polynomial p(x) can be written as a linear combination of its
basis elements p(x) = ∑ᵢ cᵢ * ϕᵢ(x), where ϕᵢ(x) is the i'th element
in the basis, cᵢ is the coefficient of that element. The most commonly
used basis is monomials. For example in polynomial p(x) = 2x₀²x₁ +
3x₀x₁ + 2, x₀²x₁, x₀x₁ and 1 are all elements of monomial basis.
Likewise, a polynomial can be written using other basis, such as
Chebyshev polynomials, Legendre polynomials, etc. For a polynomial
written with Chebyshev polynomial basis p(x) = 2T₂(x₀)T₁(x₁) + 3T₁(x₁)
+ 2T₂(x₀), T₂(x₀)T₁(x₁),T₁(x₁), and T₂(x₀) are all elements of
Chebyshev basis. This PolynomialBasisElement class represents an
element ϕᵢ(x) in the basis. We can think of an element of polynomial
basis as a mapping from the variable to its degree. So for monomial
basis element x₀²x₁, it can be thought of as a mapping {x₀ -> 2, x₁ ->
1}. For a Chebyshev basis element T₂(x₀)T₁(x₁), it can be thought of
as a mapping {x₀ -> 2, x₁ -> 1}.

Each of the derived class, ``Derived``, should implement the following
functions

- std∷map<Derived, double> operator*(const Derived& A, const Derived&B)
- std∷map<Derived, double> Derived∷Differentiate(const Variable& var)
const;
- std∷map<Derived, double> Derived∷Integrate(const Variable& var) const;
- bool Derived∷operator<(const Derived& other) const;
- std∷pair<double, Derived> EvaluatePartial(const Environment& e) const;
- void MergeBasisElementInPlace(const Derived& other)

The function lexicographical_compare can be used when implementing
operator<. The function DoEvaluatePartial can be used when
implementing EvaluatePartial)""";
        // Symbol: drake::symbolic::PolynomialBasisElement::DoEvaluatePartial
        struct /* DoEvaluatePartial */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } DoEvaluatePartial;
        // Symbol: drake::symbolic::PolynomialBasisElement::DoMergeBasisElementInPlace
        struct /* DoMergeBasisElementInPlace */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Merge this basis element with another basis element by merging their
var_to_degree_map. After merging, the degree of each variable is
raised to the sum of the degree in each basis element (if a variable
does not show up in either one of the basis element, we regard its
degree to be 0).)""";
        } DoMergeBasisElementInPlace;
        // Symbol: drake::symbolic::PolynomialBasisElement::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } EqualTo;
        // Symbol: drake::symbolic::PolynomialBasisElement::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Evaluates under a given environment ``env``.

Raises:
    RuntimeError exception if there is a variable in this monomial
    whose assignment is not provided by ``env``.)""";
        } Evaluate;
        // Symbol: drake::symbolic::PolynomialBasisElement::GetVariables
        struct /* GetVariables */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } GetVariables;
        // Symbol: drake::symbolic::PolynomialBasisElement::PolynomialBasisElement
        struct /* ctor */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc_0args =
R"""(Constructs a polynomial basis with empty var_to_degree map. This
element should be interpreted as 1.)""";
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc_1args =
R"""(Constructs a polynomial basis given the variable and the degree of
that variable.

Raises:
    RuntimeError if any of the degree is negative.

Note:
    we will ignore the variable with degree 0.)""";
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc_2args =
R"""(Constructs a polynomial basis, such that it contains the
variable-to-degree map vars(i)→degrees(i).

Raises:
    RuntimeError if ``vars`` contains repeated variables.

Raises:
    RuntimeError if any degree is negative.)""";
        } ctor;
        // Symbol: drake::symbolic::PolynomialBasisElement::ToExpression
        struct /* ToExpression */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } ToExpression;
        // Symbol: drake::symbolic::PolynomialBasisElement::degree
        struct /* degree */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Returns the degree of this PolynomialBasisElement in a variable ``v``.
If ``v`` is not a variable in this PolynomialBasisElement, then
returns 0.)""";
        } degree;
        // Symbol: drake::symbolic::PolynomialBasisElement::get_mutable_total_degree
        struct /* get_mutable_total_degree */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } get_mutable_total_degree;
        // Symbol: drake::symbolic::PolynomialBasisElement::get_mutable_var_to_degree_map
        struct /* get_mutable_var_to_degree_map */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } get_mutable_var_to_degree_map;
        // Symbol: drake::symbolic::PolynomialBasisElement::get_powers
        struct /* get_powers */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Returns variable to degree map. TODO(hongkai.dai): this function is
added because Monomial class has get_powers() function. We will remove
this get_powers() function when Monomial class is deprecated.)""";
        } get_powers;
        // Symbol: drake::symbolic::PolynomialBasisElement::lexicographical_compare
        struct /* lexicographical_compare */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Compares two PolynomialBasisElement using lexicographical order. This
function is meant to be called by the derived class, to compare two
polynomial basis of the same derived class.)""";
        } lexicographical_compare;
        // Symbol: drake::symbolic::PolynomialBasisElement::operator!=
        struct /* operator_ne */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } operator_ne;
        // Symbol: drake::symbolic::PolynomialBasisElement::total_degree
        struct /* total_degree */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc =
R"""(Returns the total degree of a polynomial basis. This is the summation
of the degree for each variable.)""";
        } total_degree;
        // Symbol: drake::symbolic::PolynomialBasisElement::var_to_degree_map
        struct /* var_to_degree_map */ {
          // Source: drake/common/symbolic/polynomial_basis_element.h
          const char* doc = R"""()""";
        } var_to_degree_map;
      } PolynomialBasisElement;
      // Symbol: drake::symbolic::RationalFunction
      struct /* RationalFunction */ {
        // Source: drake/common/symbolic/rational_function.h
        const char* doc =
R"""(Represents symbolic rational function. A function f(x) is a rational
function, if f(x) = p(x) / q(x), where both p(x) and q(x) are
polynomials of x. Note that rational functions are closed under (+, -,
x, /). One application of rational function is in polynomial
optimization, where we represent (or approximate) functions using
rational functions, and then convert the constraint f(x) = h(x) (where
h(x) is a polynomial) to a polynomial constraint p(x) - q(x) * h(x) =
0, or convert the inequality constraint f(x) >= h(x) as p(x) - q(x) *
h(x) >= 0 if we know q(x) > 0.

This class represents a special subset of the symbolic∷Expression.
While a symbolic∷Expression can represent a rational function,
extracting the numerator and denominator, generally, is quite
difficult; for instance, from p1(x) / q1(x) + p2(x) / q2(x) + ... +
pn(x) / qn(x). This class's explicit structure facilitates this
decomposition.)""";
        // Symbol: drake::symbolic::RationalFunction::EqualTo
        struct /* EqualTo */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc =
R"""(Returns true if this rational function and f are structurally equal.)""";
        } EqualTo;
        // Symbol: drake::symbolic::RationalFunction::Evaluate
        struct /* Evaluate */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc =
R"""(Evaluates this rational function under a given environment ``env``.

Raises:
    RuntimeError if there is a variable in this rational function
    whose assignment is not provided by ``env``.)""";
        } Evaluate;
        // Symbol: drake::symbolic::RationalFunction::RationalFunction
        struct /* ctor */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_0args =
R"""(Constructs a zero rational function 0 / 1.)""";
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_2args_numerator_denominator =
R"""(Constructs the rational function: numerator / denominator.

Parameter ``numerator``:
    The numerator of the fraction.

Parameter ``denominator``:
    The denominator of the fraction.

Precondition:
    denominator is not equal to zero.

Precondition:
    None of the indeterminates in the numerator can be decision
    variables in the denominator; similarly none of the indeterminates
    in the denominator can be decision variables in the numerator.)""";
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_1args_p =
R"""(Constructs the rational function: p / 1. Note that we use 1 as the
denominator.

Parameter ``p``:
    The numerator of the rational function.)""";
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_1args_m =
R"""(Constructs the rational function: m / 1 for any type which can be cast
to a monomial

Parameter ``m``:
    The numerator of the rational function.)""";
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_1args_c =
R"""(Constructs the rational function: c / 1. Note that we use 1 as the
denominator.

Parameter ``c``:
    The numerator of the rational function.)""";
        } ctor;
        // Symbol: drake::symbolic::RationalFunction::SetIndeterminates
        struct /* SetIndeterminates */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc =
R"""(Sets the indeterminates of the numerator and denominator polynomials)""";
        } SetIndeterminates;
        // Symbol: drake::symbolic::RationalFunction::ToExpression
        struct /* ToExpression */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc =
R"""(Returns an equivalent symbolic expression of this rational function.)""";
        } ToExpression;
        // Symbol: drake::symbolic::RationalFunction::denominator
        struct /* denominator */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc = R"""(Getter for the denominator.)""";
        } denominator;
        // Symbol: drake::symbolic::RationalFunction::numerator
        struct /* numerator */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc = R"""(Getter for the numerator.)""";
        } numerator;
        // Symbol: drake::symbolic::RationalFunction::operator!=
        struct /* operator_ne */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc =
R"""(Returns a symbolic formula representing the condition where this
rational function and ``f`` are not the same.)""";
        } operator_ne;
        // Symbol: drake::symbolic::RationalFunction::operator*=
        struct /* operator_imul */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc = R"""()""";
        } operator_imul;
        // Symbol: drake::symbolic::RationalFunction::operator+=
        struct /* operator_iadd */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc = R"""()""";
        } operator_iadd;
        // Symbol: drake::symbolic::RationalFunction::operator-=
        struct /* operator_isub */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc = R"""()""";
        } operator_isub;
        // Symbol: drake::symbolic::RationalFunction::operator/=
        struct /* operator_idiv */ {
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_1args_f =
R"""(Raises:
    RuntimeError if the numerator of the divisor is structurally equal
    to zero. Note that this does not guarantee that the denominator of
    the result is not zero after expansion.)""";
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_1args_p =
R"""(Raises:
    RuntimeError if the divisor is structurally equal to zero. Note
    that this does not guarantee that the denominator of the result is
    not zero after expansion.)""";
          // Source: drake/common/symbolic/rational_function.h
          const char* doc_1args_c =
R"""(Raises:
    RuntimeError if c is 0)""";
        } operator_idiv;
      } RationalFunction;
      // Symbol: drake::symbolic::ReplaceBilinearTerms
      struct /* ReplaceBilinearTerms */ {
        // Source: drake/common/symbolic/replace_bilinear_terms.h
        const char* doc =
R"""(Replaces all the bilinear product terms in the expression ``e``, with
the corresponding terms in ``W``, where ``W`` represents the matrix x
* yᵀ, such that after replacement, ``e`` does not have bilinear terms
involving ``x`` and ``y``. For example, if e = x(0)*y(0) + 2 *
x(0)*y(1) + x(1) * y(1) + 3 * x(1), ``e`` has bilinear terms
x(0)*y(0), x(0) * y(1) and x(2) * y(1), if we call
ReplaceBilinearTerms(e, x, y, W) where W(i, j) represent the term x(i)
* y(j), then this function returns W(0, 0) + 2 * W(0, 1) + W(1, 1) + 3
* x(1).

Parameter ``e``:
    An expression potentially contains bilinear products between x and
    y.

Parameter ``x``:
    The bilinear product between ``x`` and ``y`` will be replaced by
    the corresponding term in ``W``.

Raises:
    RuntimeError if ``x`` contains duplicate entries.

Parameter ``y``:
    The bilinear product between ``x`` and ``y`` will be replaced by
    the corresponding term in ``W``.

Raises:
    RuntimeError if ``y`` contains duplicate entries.

Parameter ``W``:
    Bilinear product term x(i) * y(j) will be replaced by W(i, j). If
    W(i,j) is not a single variable, but an expression, then this
    expression cannot contain a variable in either x or y.

Raises:
    RuntimeError, if W(i, j) is not a single variable, and also
    contains a variable in x or y.

Precondition:
    W.rows() == x.rows() and W.cols() == y.rows().

Returns:
    The symbolic expression after replacing x(i) * y(j) with W(i, j).)""";
      } ReplaceBilinearTerms;
      // Symbol: drake::symbolic::Rewriter
      struct /* Rewriter */ {
        // Source: drake/common/symbolic/simplification.h
        const char* doc =
R"""(A ``Rewriter`` is a function from an Expression to an Expression.)""";
      } Rewriter;
      // Symbol: drake::symbolic::RewritingRule
      struct /* RewritingRule */ {
        // Source: drake/common/symbolic/simplification.h
        const char* doc =
R"""(A ``RewritingRule``, `lhs => rhs`, consists of two Patterns ``lhs``
and ``rhs``. A rewriting rule instructs a rewriter how to transform a
given expression ``e``. First, the rewriter tries to find a match
between the expression ``e`` and the pattern ``lhs``. If such a match
is found, it applies the match result (substitution) to ``rhs``.
Otherwise, the same expression ``e`` is returned.)""";
        // Symbol: drake::symbolic::RewritingRule::RewritingRule
        struct /* ctor */ {
          // Source: drake/common/symbolic/simplification.h
          const char* doc =
R"""(Constructs a rewriting rule ``lhs => rhs``.)""";
          // Source: drake/common/symbolic/simplification.h
          const char* doc_copy = R"""(Default copy constructor.)""";
          // Source: drake/common/symbolic/simplification.h
          const char* doc_move = R"""(Default move constructor.)""";
        } ctor;
        // Symbol: drake::symbolic::RewritingRule::lhs
        struct /* lhs */ {
          // Source: drake/common/symbolic/simplification.h
          const char* doc =
R"""(Returns the const reference of the LHS of the rewriting rule.)""";
        } lhs;
        // Symbol: drake::symbolic::RewritingRule::rhs
        struct /* rhs */ {
          // Source: drake/common/symbolic/simplification.h
          const char* doc =
R"""(Returns the const reference of the RHS of the rewriting rule.)""";
        } rhs;
      } RewritingRule;
      // Symbol: drake::symbolic::SinCos
      struct /* SinCos */ {
        // Source: drake/common/symbolic/trigonometric_polynomial.h
        const char* doc =
R"""(Represents a pair of Variables corresponding to sin(q) and cos(q).)""";
        // Symbol: drake::symbolic::SinCos::SinCos
        struct /* ctor */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::symbolic::SinCos::c
        struct /* c */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc = R"""(cos variable.)""";
        } c;
        // Symbol: drake::symbolic::SinCos::s
        struct /* s */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc = R"""(sin variable.)""";
        } s;
        // Symbol: drake::symbolic::SinCos::type
        struct /* type */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc =
R"""(Allows a user to specify non-default substitutions, such as using
half-angle formulas.)""";
        } type;
      } SinCos;
      // Symbol: drake::symbolic::SinCosSubstitution
      struct /* SinCosSubstitution */ {
        // Source: drake/common/symbolic/trigonometric_polynomial.h
        const char* doc = R"""()""";
      } SinCosSubstitution;
      // Symbol: drake::symbolic::SinCosSubstitutionType
      struct /* SinCosSubstitutionType */ {
        // Source: drake/common/symbolic/trigonometric_polynomial.h
        const char* doc = R"""()""";
        // Symbol: drake::symbolic::SinCosSubstitutionType::kAngle
        struct /* kAngle */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc = R"""(Substitutes s <=> sin(q), c <=> cos(q).)""";
        } kAngle;
        // Symbol: drake::symbolic::SinCosSubstitutionType::kHalfAnglePreferCos
        struct /* kHalfAnglePreferCos */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc =
R"""(Substitutes s <=> sin(q/2), c <=> cos(q/2), and prefers cos when the
choice is ambiguous; e.g. cos(q) => 2c² - 1.)""";
        } kHalfAnglePreferCos;
        // Symbol: drake::symbolic::SinCosSubstitutionType::kHalfAnglePreferSin
        struct /* kHalfAnglePreferSin */ {
          // Source: drake/common/symbolic/trigonometric_polynomial.h
          const char* doc =
R"""(Substitutes s <=> sin(q/2), c <=> cos(q/2), and prefers sin when the
choice is ambiguous; e.g. cos(q) => 1 - 2s².)""";
        } kHalfAnglePreferSin;
      } SinCosSubstitutionType;
      // Symbol: drake::symbolic::Substitute
      struct /* Substitute */ {
        // Source: drake/common/symbolic/trigonometric_polynomial.h
        const char* doc_sincos =
R"""(Given a substitution map q => {s, c}, substitutes instances of sin(q)
and cos(q) in ``e`` with ``s`` and ``c``, with partial support for
trigonometric expansions. For instance,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Variable x{"x"}, y{"y"};
    Variable sx{"sx"}, cx{"cx"}, sy{"sy"}, cy{"cy"};
    SinCosSubstitution subs;
    subs.emplace(x, SinCos(sx, cx));
    subs.emplace(y, SinCos(sy, cy));
    Expression e = Substitute(x * sin(x + y), subs);

.. raw:: html

    </details>

will result in the expression ``x * (sx*cy + cx*sy)``.

Parameter ``subs``:
    When set to one of the half_angle options, then the same workflow
    replaces instances of sin(q/2) and cos(q/2) in ``e`` will be
    replaced with ``s``, and ``c``.

*Default:* false.
    The half-angle representation is more natural in many analysis
    computations for robots, for instance:
    https://underactuated.csail.mit.edu/lyapunov.html#trig_quadratic

Raises:
    RuntimeError if a trigonometric function is not a trigonometric
    polynomial in ``q`` or if the ``e`` requires a trigonometric
    expansion that not supported yet.)""";
        // Source: drake/common/symbolic/trigonometric_polynomial.h
        const char* doc_sincos_matrix =
R"""(Matrix version of sin/cos substitution.)""";
      } Substitute;
      // Symbol: drake::symbolic::SubstituteStereographicProjection
      struct /* SubstituteStereographicProjection */ {
        // Source: drake/common/symbolic/trigonometric_polynomial.h
        const char* doc =
R"""(Substitutes the variables representing sine and cosine functions with
their stereographic projection. We replace cosθᵢ with (1-tᵢ²)/(1+tᵢ²),
and sinθᵢ with 2tᵢ/(1+tᵢ²), and get a rational polynomial. The
indeterminates of this rational polynomial are t together with the
indeterminates in ``e`` that are not cosθ or sinθ. If the input
expression doesn't contain the sine and cosine functions, then the
returned rational has denominator being 1. Notice that the
indeterminates of ``e`` can include variables other than cosθ and
sinθ, and we impose no requirements on these variables that are not
cosθ or sinθ.

Parameter ``e``:
    The symbolic polynomial to be substituted.

Parameter ``sin_cos``:
    sin_cos(i) is the pair of variables (sᵢ, cᵢ), (where sᵢ=sinθᵢ,
    cᵢ=cosθᵢ) as documented above.

Parameter ``t``:
    New variables to express cos and sin as rationals of t. tᵢ =
    tan(θᵢ/2).

Precondition:
    t.rows() == sin_cos.size()

Returns:
    e_rational The rational polynomial of e after replacement. The
    indeterminates of the polynomials are ``t`` together with the
    indeterminates in ``e`` that are not cosθ or sinθ. Example


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    std∷vector<SinCos> sin_cos;
    sin_cos.emplace_back(symbolic∷Variable("s0"), symbolic∷Variable("c0"));
    sin_cos.emplace_back(symbolic∷Variable("s1"), symbolic∷Variable("c1"));
    Vector2<symbolic∷Variable> t(symbolic∷Variable("t0"),
                                  symbolic∷Variable("t1"));
    const auto e_rational =
    SubstituteStereographicProjection(t(0) * sin_cos[0].s*sin_cos[1].c + 1,
                                      sin_cos, t);
    // e_rational should be
    // (2*t0*t0*(1-t1*t1) + (1+t0*t0)*(1+t1*t1))
    // --------------------------------------------
    //        ((1+t0*t0)*(1+t1*t1))

.. raw:: html

    </details>)""";
      } SubstituteStereographicProjection;
      // Symbol: drake::symbolic::ToLatex
      struct /* ToLatex */ {
        // Source: drake/common/symbolic/latex.h
        const char* doc_expression =
R"""(Generates a LaTeX string representation of ``e`` with floating point
coefficients displayed using ``precision``.)""";
        // Source: drake/common/symbolic/latex.h
        const char* doc_formula =
R"""(Generates a LaTeX string representation of ``f`` with floating point
coefficients displayed using ``precision``.)""";
        // Source: drake/common/symbolic/latex.h
        const char* doc =
R"""(Generates a Latex string representation of ``val`` displayed with
``precision``, with one exception. If the fractional part of ``val``
is exactly zero, then ``val`` is represented perfectly as an integer,
and is displayed without the trailing decimal point and zeros (in this
case, the ``precision`` argument is ignored).)""";
        // Source: drake/common/symbolic/latex.h
        const char* doc_matrix =
R"""(Generates a LaTeX string representation of ``M`` with floating point
coefficients displayed using ``precision``.)""";
      } ToLatex;
      // Symbol: drake::symbolic::operator*
      struct /* operator_mul */ {
        // Source: drake/common/symbolic/chebyshev_basis_element.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Returns the product of two Chebyshev basis elements. Since Tₘ(x) *
Tₙ(x) = 0.5 (Tₘ₊ₙ(x) + Tₘ₋ₙ(x)) if m >= n, the product of Chebyshev
basis elements is the weighted sum of several Chebyshev basis
elements. For example T₁(x)T₂(y) * T₃(x)T₁(y) = 0.25*(T₄(x)T₃(y) +
T₂(x)T₃(y) + T₄(x)T₁(y) + T₂(x)T₁(y))

Returns:
    the result of the product, from each ChebyshevBasisElement to its
    coefficient. In the example above, it returns (T₄(x)T₃(y) ->
    0.25), (T₂(x)T₃(y) -> 0.25), (T₄(x)T₁(y) -> 0.25) and (T₂(x)T₁(y)
    -> 0.25))""";
      } operator_mul;
      // Symbol: drake::symbolic::operator+
      struct /* operator_add */ {
        // Source: drake/common/symbolic/generic_polynomial.h
        const char* doc = R"""()""";
      } operator_add;
      // Symbol: drake::symbolic::operator-
      struct /* operator_sub */ {
        // Source: drake/common/symbolic/polynomial.h
        const char* doc = R"""(Unary minus operation for polynomial.)""";
      } operator_sub;
      // Symbol: drake::symbolic::operator/
      struct /* operator_div */ {
        // Source: drake/common/symbolic/generic_polynomial.h
        const char* doc_2args_GenericPolynomial_double = R"""(Returns ``p / v``.)""";
        // Source: drake/common/symbolic/rational_function.h
        const char* doc_2args_f1_f2 =
R"""(Raises:
    RuntimeError if the numerator of the divisor is structurally equal
    to zero. Note that this does not guarantee that the denominator of
    the result is not zero after expansion.)""";
        // Source: drake/common/symbolic/rational_function.h
        const char* doc_2args_f_p =
R"""(Raises:
    RuntimeError if the divisor is structurally equal to zero. Note
    that this does not guarantee that the denominator of the result is
    not zero after expansion.)""";
        // Source: drake/common/symbolic/rational_function.h
        const char* doc_2args_f_c =
R"""(Raises:
    RuntimeError if c is 0)""";
      } operator_div;
      // Symbol: drake::symbolic::pow
      struct /* pow */ {
        // Source: drake/common/symbolic/generic_polynomial.h
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Returns polynomial ``raised`` to ``n``.

Parameter ``p``:
    The base polynomial.

Parameter ``n``:
    The exponent of the power.

Precondition:
    n>=0.)""";
      } pow;
      // Symbol: drake::symbolic::to_string
      struct /* to_string */ {
        // Source: drake/common/symbolic/chebyshev_basis_element.h
        const char* doc = R"""()""";
      } to_string;
    } symbolic;
  } drake;
} pydrake_doc_common_symbolic;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
