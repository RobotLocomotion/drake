#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/symbolic/monomial_util.h"
#include "drake/common/symbolic/polynomial.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/function.h"
#include "drake/solvers/indeterminate.h"
#include "drake/solvers/program_attribute.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {

template <int...>
struct NewVariableNames {};
/**
 * The type of the names for the newly added variables.
 * @tparam Size If Size is a fixed non-negative integer, then the type of the
 * name is std::array<std::string, Size>. Otherwise the type is
 * std::vector<std::string>.
 */
template <int Size>
struct NewVariableNames<Size> {
  typedef std::array<std::string, Size> type;
};

template <>
struct NewVariableNames<Eigen::Dynamic> {
  typedef std::vector<std::string> type;
};

template <int Rows, int Cols>
struct NewVariableNames<Rows, Cols>
    : public NewVariableNames<
          Eigen::Matrix<double, Rows, Cols>::SizeAtCompileTime> {};

template <int Rows>
struct NewSymmetricVariableNames
    : public NewVariableNames<Rows == Eigen::Dynamic ? Eigen::Dynamic
                                                     : Rows*(Rows + 1) / 2> {};

namespace internal {
/**
 * Return un-initialized new variable names.
 */
template <int Size>
  requires(Size >= 0)
typename NewVariableNames<Size>::type CreateNewVariableNames(int) {
  typename NewVariableNames<Size>::type names;
  return names;
}

/**
 * Return un-initialized new variable names.
 */
template <int Size>
  requires(Size == Eigen::Dynamic)
typename NewVariableNames<Size>::type CreateNewVariableNames(int size) {
  typename NewVariableNames<Eigen::Dynamic>::type names(size);
  return names;
}
/**
 * Set the names of the newly added variables.
 * @param name The common name of all new variables.
 * @param rows The number of rows in the new variables.
 * @param cols The number of columns in the new variables.
 * @pre The size of @p names is @p rows * @p cols.
 */
template <typename Derived>
void SetVariableNames(const std::string& name, int rows, int cols,
                      Derived* names) {
  DRAKE_DEMAND(static_cast<int>(names->size()) == rows * cols);
  if (cols == 1) {
    for (int i = 0; i < rows; ++i) {
      (*names)[i] = name + "(" + std::to_string(i) + ")";
    }
  } else {
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        (*names)[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
  }
}

/**
 * Template condition to only catch when Constraints are inadvertently passed
 * as an argument. If the class is binding-compatible with a Constraint, then
 * this will provide a static assertion to enable easier debugging of which
 * type failed.
 * @tparam F The type to be tested.
 * @see http://stackoverflow.com/a/13366183/7829525
 */
template <typename F>
struct assert_if_is_constraint {
  static constexpr bool value = is_binding_compatible<F, Constraint>::value;
  // Use deferred evaluation
  static_assert(
      !value,
      "You cannot pass a Constraint to create a Cost object from a function. "
      "Please ensure you are passing a Cost.");
};
}  // namespace internal

/**
 * MathematicalProgram stores the decision variables, the constraints and costs
 * of an optimization problem. The user can solve the problem by calling
 * solvers::Solve() function, and obtain the results of the optimization.
 *
 * @ingroup solvers
 */
class MathematicalProgram {
 public:
  /** @name Does not allow copy, move, or assignment. */
  /** @{ */
#ifdef DRAKE_DOXYGEN_CXX
  // Copy constructor is private for use in implementing Clone().
  MathematicalProgram(const MathematicalProgram&) = delete;
#endif
  MathematicalProgram& operator=(const MathematicalProgram&) = delete;
  MathematicalProgram(MathematicalProgram&&) = delete;
  MathematicalProgram& operator=(MathematicalProgram&&) = delete;
  /** @} */

  using VarType = symbolic::Variable::Type;

  /// The optimal cost is +∞ when the problem is globally infeasible.
  static constexpr double kGlobalInfeasibleCost =
      std::numeric_limits<double>::infinity();
  /// The optimal cost is -∞ when the problem is unbounded.
  static constexpr double kUnboundedCost =
      -std::numeric_limits<double>::infinity();

  MathematicalProgram();
  virtual ~MathematicalProgram();

  /** Clones an optimization program.
   * The clone will be functionally equivalent to the source program with the
   * same:
   *
   * - decision variables
   * - constraints
   * - costs
   * - solver settings
   * - initial guess
   *
   * Note that this is currently a *shallow* clone. The costs and constraints
   * are not themselves cloned.
   *
   * @retval new_prog. The newly constructed mathematical program.
   */
  [[nodiscard]] std::unique_ptr<MathematicalProgram> Clone() const;

  /**
   * Returns string representation of this program, listing the decision
   * variables, costs, and constraints.
   *
   * Note that by default, we do not require variables to have unique names.
   * Providing useful variable names and calling Evaluator::set_description() to
   * describe the costs and constraints can dramatically improve the readability
   * of the output.  See the tutorial `debug_mathematical_program.ipynb`
   * for more information.
   */
  [[nodiscard]] std::string to_string() const;

  /**
   * Returns whether it is safe to solve this mathematical program concurrently.
   * A mathematical program is safe to solve concurrently if all of its cost,
   * constraints, and visualization callbacks are marked as thread safe.
   */
  bool IsThreadSafe() const;

  /** Returns a string representation of this program in LaTeX.
   *
   * This can be particularly useful e.g. in a Jupyter (python) notebook:
   * @code
   * from IPython.display import Markdown, display
   * display(Markdown(prog.ToLatex()))
   * @endcode
   *
   * Note that by default, we do not require variables to have unique names.
   * Providing useful variable names and calling Evaluator::set_description() to
   * describe the costs and constraints can dramatically improve the readability
   * of the output.  See the tutorial `debug_mathematical_program.ipynb`
   * for more information.
   */
  std::string ToLatex(int precision = 3);

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @param rows  The number of rows in the new variables.
   * @param name The name of the newly added variables
   * @return The VectorDecisionVariable of size rows x 1, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, "x");
   * @endcode
   * This adds a 2 x 1 vector containing decision variables into the program.
   * The names of the variables are "x(0)" and "x(1)".
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  VectorXDecisionVariable NewContinuousVariables(
      int rows, const std::string& name = "x") {
    return NewContinuousVariables<Eigen::Dynamic, 1>(rows, 1, name);
  }

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows The number of rows of the new variables, in the compile time.
   * @tparam Cols The number of columns of the new variables, in the compile
   * time.
   * @param rows The number of rows in the new variables. When Rows is not
   * Eigen::Dynamic, rows is ignored.
   * @param cols The number of columns in the new variables. When Cols is not
   * Eigen::Dynamic, cols is ignored.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixDecisionVariable of size Rows x Cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables(2, 3, "X");
   * auto y = prog.NewContinuousVariables<2, 3>(2, 3, "X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
  MatrixDecisionVariable<Rows, Cols> NewContinuousVariables(
      int rows, int cols, const std::string& name = "X") {
    rows = Rows == Eigen::Dynamic ? rows : Rows;
    cols = Cols == Eigen::Dynamic ? cols : Cols;
    auto names = internal::CreateNewVariableNames<
        Eigen::Matrix<double, Rows, Cols>::SizeAtCompileTime>(rows * cols);
    internal::SetVariableNames(name, rows, cols, &names);
    return NewVariables<Rows, Cols>(VarType::CONTINUOUS, names, rows, cols);
  }

  /**
   * Adds continuous variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows  The number of rows in the new variables.
   * @tparam Cols  The number of columns in the new variables. The default is 1.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows, int Cols = 1>
  MatrixDecisionVariable<Rows, Cols> NewContinuousVariables(
      const std::string& name = "X") {
    return NewContinuousVariables<Rows, Cols>(Rows, Cols, name);
  }

  /**
   * Adds binary variables, appending them to an internal vector of any
   * existing vars.
   * The initial guess values for the new variables are set to NaN, to
   * indicate that an initial guess has not been assigned.
   * Callers are expected to add costs
   * and/or constraints to have any effect during optimization.
   * Callers can also set the initial guess of the decision variables through
   * SetInitialGuess() or SetInitialGuessForAllVariables().
   * @tparam Rows  The number of rows in the new variables.
   * @tparam Cols  The number of columns in the new variables.
   * @param rows The number of rows in the new variables.
   * @param cols The number of columns in the new variables.
   * @param name The commonly shared name of the new variables.
   * @return The MatrixDecisionVariable of size rows x cols, containing the new
   * vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto b = prog.NewBinaryVariables(2, 3, "b");
   * @endcode
   * This adds a 2 x 3 matrix decision variables into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   */
  template <int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
  MatrixDecisionVariable<Rows, Cols> NewBinaryVariables(
      int rows, int cols, const std::string& name) {
    rows = Rows == Eigen::Dynamic ? rows : Rows;
    cols = Cols == Eigen::Dynamic ? cols : Cols;
    auto names = internal::CreateNewVariableNames<
        Eigen::Matrix<double, Rows, Cols>::SizeAtCompileTime>(rows * cols);
    internal::SetVariableNames(name, rows, cols, &names);
    return NewVariables<Rows, Cols>(VarType::BINARY, names, rows, cols);
  }

  /**
   * Adds a matrix of binary variables into the optimization program.
   * @tparam Rows The number of rows in the newly added binary variables.
   * @tparam Cols  The number of columns in the new variables. The default is 1.
   * @param name Each newly added binary variable will share the same name. The
   * default name is "b".
   * @return A matrix containing the newly added variables.
   */
  template <int Rows, int Cols = 1>
  MatrixDecisionVariable<Rows, Cols> NewBinaryVariables(
      const std::string& name = "b") {
    return NewBinaryVariables<Rows, Cols>(Rows, Cols, name);
  }

  /**
   * Adds binary variables to this MathematicalProgram. The new variables are
   * viewed as a column vector, with size @p rows x 1.
   * @see NewBinaryVariables(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXDecisionVariable NewBinaryVariables(int rows,
                                             const std::string& name = "b") {
    return NewBinaryVariables<Eigen::Dynamic, 1>(rows, 1, name);
  }

  /**
   * Adds a runtime sized symmetric matrix as decision variables to
   * this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @param rows The number of rows in the symmetric matrix.
   * @param name The name of the matrix. It is only used the for user to
   * understand the optimization program. The default name is "Symmetric", and
   * each variable will be named as
   * <pre>
   * Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
   * Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
   *            ...
   * Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)
   * </pre>
   * Notice that the (i,j)'th entry and (j,i)'th entry has the same name.
   * @return The newly added decision variables.
   */
  MatrixXDecisionVariable NewSymmetricContinuousVariables(
      int rows, const std::string& name = "Symmetric");

  /**
   * Adds a static sized symmetric matrix as decision variables to
   * this MathematicalProgram.
   * The optimization will only use the stacked columns of the
   * lower triangular part of the symmetric matrix as decision variables.
   * @tparam rows The number of rows in the symmetric matrix.
   * @param name The name of the matrix. It is only used the for user to
   * understand the optimization program. The default name is "Symmetric", and
   * each variable will be named as
   * <pre>
   * Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
   * Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
   *            ...
   * Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)
   * </pre>
   * Notice that the (i,j)'th entry and (j,i)'th entry has the same name.
   * @return The newly added decision variables.
   */
  template <int rows>
  MatrixDecisionVariable<rows, rows> NewSymmetricContinuousVariables(
      const std::string& name = "Symmetric") {
    typename NewSymmetricVariableNames<rows>::type names;
    int var_count = 0;
    for (int j = 0; j < static_cast<int>(rows); ++j) {
      for (int i = j; i < static_cast<int>(rows); ++i) {
        names[var_count] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
        ++var_count;
      }
    }
    return NewSymmetricVariables<rows>(VarType::CONTINUOUS, names);
  }

  /** Appends new variables to the end of the existing variables.
   * @param decision_variables The newly added decision_variables.
   * @pre `decision_variables` should not intersect with the existing
   * indeterminates in the optimization program.
   * @throws std::exception if the preconditions are not satisfied.
   */
  void AddDecisionVariables(
      const Eigen::Ref<const MatrixXDecisionVariable>& decision_variables);

  /**
   * Returns a free polynomial in a monomial basis over @p indeterminates of a
   * given @p degree. It uses @p coeff_name to make new decision variables and
   * use them as coefficients. For example, `NewFreePolynomial({x₀, x₁}, 2)`
   * returns a₀x₁² + a₁x₀x₁ + a₂x₀² + a₃x₁ + a₄x₀ + a₅.
   */
  symbolic::Polynomial NewFreePolynomial(
      const symbolic::Variables& indeterminates, int degree,
      const std::string& coeff_name = "a");

  /**
   * Returns a free polynomial that only contains even degree monomials. A
   * monomial is even degree if its total degree (sum of all variables' degree)
   * is even. For example, xy is an even degree monomial (degree 2) while x²y is
   * not (degree 3).
   * @param indeterminates The monomial basis is over these indeterminates.
   * @param degree The highest degree of the polynomial.
   * @param coeff_name The coefficients of the polynomial are decision variables
   * with this name as a base. The variable name would be "a1", "a2", etc.
   */
  symbolic::Polynomial NewEvenDegreeFreePolynomial(
      const symbolic::Variables& indeterminates, int degree,
      const std::string& coeff_name = "a");

  /**
   * Returns a free polynomial that only contains odd degree monomials. A
   * monomial is odd degree if its total degree (sum of all variables' degree)
   * is even. For example, xy is not an odd degree monomial (degree 2) while x²y
   * is (degree 3).
   * @param indeterminates The monomial basis is over these indeterminates.
   * @param degree The highest degree of the polynomial.
   * @param coeff_name The coefficients of the polynomial are decision variables
   * with this name as a base. The variable name would be "a1", "a2", etc.
   */
  symbolic::Polynomial NewOddDegreeFreePolynomial(
      const symbolic::Variables& indeterminates, int degree,
      const std::string& coeff_name = "a");

  /**
   * Types of non-negative polynomial that can be found through conic
   * optimization. We currently support SOS, SDSOS and DSOS. For more
   * information about these polynomial types, please refer to
   * "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586
   */
  enum class NonnegativePolynomial {
    // We reserve the 0 value as a tactic for identifying uninitialized enums.
    kSos = 1,  ///< A sum-of-squares polynomial.
    kSdsos,    ///< A scaled-diagonally dominant sum-of-squares polynomial.
    kDsos,     ///< A diagonally dominant sum-of-squares polynomial.
  };

  /** Returns a pair of a SOS polynomial p = mᵀQm and the Gramian matrix Q,
   * where m is the @p monomial basis.
   * For example, `NewSosPolynomial(Vector2<Monomial>{x,y})` returns a
   * polynomial
   *   p = Q₍₀,₀₎x² + 2Q₍₁,₀₎xy + Q₍₁,₁₎y²
   * and Q.
   * Depending on the type of the polynomial, we will impose different
   * constraint on the polynomial.
   * - if type = kSos, we impose the polynomial being SOS.
   * - if type = kSdsos, we impose the polynomial being SDSOS.
   * - if type = kDsos, we impose the polynomial being DSOS.
   * @param gram_name The name of the gram matrix for print out.
   * @note Q is a symmetric monomial_basis.rows() x monomial_basis.rows()
   * matrix.
   */
  std::pair<symbolic::Polynomial, MatrixXDecisionVariable> NewSosPolynomial(
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      NonnegativePolynomial type = NonnegativePolynomial::kSos,
      const std::string& gram_name = "S");

  /**
   * Overloads NewSosPolynomial, except the Gramian matrix Q is an
   * input instead of an output.
   */
  symbolic::Polynomial NewSosPolynomial(
      const Eigen::Ref<const MatrixX<symbolic::Variable>>& gramian,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      NonnegativePolynomial type = NonnegativePolynomial::kSos);

  /**
   * Overloads NewSosPolynomial.
   * Returns a pair of a SOS polynomial p = m(x)ᵀQm(x) of degree @p degree
   * and the Gramian matrix Q that should be PSD, where m(x) is the
   * result of calling `MonomialBasis(indeterminates, degree/2)`. For example,
   * `NewSosPolynomial({x}, 4)` returns a pair of a polynomial
   *   p = Q₍₀,₀₎x⁴ + 2Q₍₁,₀₎ x³ + (2Q₍₂,₀₎ + Q₍₁,₁₎)x² + 2Q₍₂,₁₎x + Q₍₂,₂₎
   * and Q.
   * @param type Depending on the type of the polynomial, we will impose
   * different constraint on the polynomial.
   * - if type = kSos, we impose the polynomial being SOS.
   * - if type = kSdsos, we impose the polynomial being SDSOS.
   * - if type = kDsos, we impose the polynomial being DSOS.
   * @param gram_name The name of the gram matrix for print out.
   *
   * @throws std::exception if @p degree is not a positive even integer.
   * @see MonomialBasis.
   */
  std::pair<symbolic::Polynomial, MatrixXDecisionVariable> NewSosPolynomial(
      const symbolic::Variables& indeterminates, int degree,
      NonnegativePolynomial type = NonnegativePolynomial::kSos,
      const std::string& gram_name = "S");

  /**
   * @anchor even_degree_nonnegative_polynomial
   * @name    Creating even-degree nonnegative polynomials
   * Creates a nonnegative polynomial p = m(x)ᵀQm(x) of a given degree, and p
   * only contains even-degree monomials. If we partition the monomials m(x) to
   * odd degree monomials m_o(x) and even degree monomials m_e(x), then we
   * can write p(x) as
   * <pre>
   * ⌈m_o(x)⌉ᵀ * ⌈Q_oo Q_oeᵀ⌉ * ⌈m_o(x)⌉
   * ⌊m_e(x)⌋    ⌊Q_oe Q_ee ⌋   ⌊m_e(x)⌋
   * </pre>
   * Since p(x) doesn't contain any odd degree monomials, and p(x) contains
   * terms m_e(x)ᵀ*Q_oe * m_o(x) which has odd degree, we know that the
   * off-diagonal block Q_oe has to be zero. So the constraint that Q is psd
   * can be simplified as Q_oo and Q_ee has to be psd. Since both Q_oo and
   * Q_ee have smaller size than Q, these PSD constraints are easier to solve
   * than requiring Q to be PSD.
   * One use case for even-degree polynomial, is for polynomials that are even
   * functions, namely p(x) = p(-x).
   * @param indeterminates The set of indeterminates x
   * @param degree The total degree of the polynomial p. @pre This must be an
   * even number.
   * @return (p(x), Q_oo, Q_ee). p(x) is the newly added non-negative
   * polynomial. p(x) = m_o(x)ᵀ*Q_oo*m_o(x) + m_e(x)ᵀ*Q_ee*m_e(x) where m_o(x)
   * and m_e(x) contain all the even/odd monomials of x respectively.
   * The returned non-negative polynomial can be of different types, including
   * Sum-of-squares (SOS), diagonally-dominant-sum-of-squares (dsos), and
   * scaled-diagonally-dominant-sum-of-squares (sdsos).
   */

  //@{
  /**
   * See @ref even_degree_nonnegative_polynomial for more details.
   * Variant that produces different non-negative polynomials depending on @p
   * type.
   * @param type The returned polynomial p(x) can be either SOS, SDSOS or DSOS,
   * depending on @p type.
   */
  auto NewEvenDegreeNonnegativePolynomial(
      const symbolic::Variables& indeterminates, int degree,
      NonnegativePolynomial type)
      -> std::tuple<symbolic::Polynomial, MatrixXDecisionVariable,
                    MatrixXDecisionVariable>;

  /**
   * See @ref even_degree_nonnegative_polynomial for more details.
   * Variant that produces a SOS polynomial.
   */
  auto NewEvenDegreeSosPolynomial(const symbolic::Variables& indeterminates,
                                  int degree)
      -> std::tuple<symbolic::Polynomial, MatrixXDecisionVariable,
                    MatrixXDecisionVariable>;

  /**
   * see @ref even_degree_nonnegative_polynomial for details.
   * Variant that produces an SDSOS polynomial.
   */
  auto NewEvenDegreeSdsosPolynomial(const symbolic::Variables& indeterminates,
                                    int degree)
      -> std::tuple<symbolic::Polynomial, MatrixXDecisionVariable,
                    MatrixXDecisionVariable>;

  /**
   * see @ref even_degree_nonnegative_polynomial for details.
   * Variant that produces a DSOS polynomial.
   * Same as NewEvenDegreeSosPolynomial, except the returned polynomial is
   * diagonally dominant sum of squares (dsos).
   */
  auto NewEvenDegreeDsosPolynomial(const symbolic::Variables& indeterminates,
                                   int degree)
      -> std::tuple<symbolic::Polynomial, MatrixXDecisionVariable,
                    MatrixXDecisionVariable>;
  //@}

  /**
   * Creates a symbolic polynomial from the given expression `e`. It uses this
   * MathematicalProgram's `indeterminates()` in constructing the polynomial.
   *
   * This method helps a user create a polynomial with the right set of
   * indeterminates which are declared in this MathematicalProgram. We recommend
   * users to use this method over an explicit call to Polynomial constructors
   * to avoid a possible mismatch between this MathematicalProgram's
   * indeterminates and the user-specified indeterminates (or unspecified, which
   * then includes all symbolic variables in the expression `e`). Consider the
   * following example.
   *
   *   e = ax + bx + c
   *
   *   MP.indeterminates()     = {x}
   *   MP.decision_variables() = {a, b}
   *
   * - `MP.MakePolynomial(e)` create a polynomial, `(a + b)x + c`.  Here only
   *   `x` is an indeterminate of this polynomial.
   *
   * - In contrast, `symbolic::Polynomial(e)` returns `ax + bx + c` where all
   *   variables `{a, b, x}` are indeterminates. Note that this is problematic
   *   as its indeterminates, `{a, b, x}` and the MathematicalProgram's decision
   *   variables, `{a, b}` overlap.
   *
   * @note This function does not require that the decision variables in `e` is
   * a subset of the decision variables in MathematicalProgram.
   */
  [[nodiscard]] symbolic::Polynomial MakePolynomial(
      const symbolic::Expression& e) const;

  /**
   * Reparses the polynomial `p` using this MathematicalProgram's
   * indeterminates.
   */
  void Reparse(symbolic::Polynomial* p) const;

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 6> names = {"x1", "x2", "x3", "x4", "x5", "x6"};
   * auto x = prog.NewIndeterminates<2, 3>(names);
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows, cols) are not bound in pydrake.}
   */
  template <int rows, int cols>
  MatrixIndeterminate<rows, cols> NewIndeterminates(
      const std::array<std::string, rows * cols>& names) {
    MatrixIndeterminate<rows, cols> indeterminates_matrix;
    NewIndeterminates_impl(names, indeterminates_matrix);
    return indeterminates_matrix;
  }

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * std::array<std::string, 2> names = {"x1", "x2"};
   * auto x = prog.NewIndeterminates<2>(names);
   * @endcode
   * This adds a 2 vector indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows) are not bound in pydrake.}
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(
      const std::array<std::string, rows>& names) {
    return NewIndeterminates<rows, 1>(names);
  }

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing indeterminates.
   * @tparam rows  The number of rows in the new indeterminates.
   * @tparam cols  The number of columns in the new indeterminates.
   * @param name All variables will share the same name, but different index.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewIndeterminates<2, 3>("X");
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the indeterminates is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows, cols) are not bound in pydrake.}
   */
  template <int rows, int cols>
  MatrixIndeterminate<rows, cols> NewIndeterminates(
      const std::string& name = "X") {
    std::array<std::string, rows * cols> names;
    for (int j = 0; j < cols; ++j) {
      for (int i = 0; i < rows; ++i) {
        names[j * rows + i] =
            name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      }
    }
    return NewIndeterminates<rows, cols>(names);
  }

  /**
   * Adds indeterminates to the program.
   * The name for all newly added indeterminates are set to @p name. The default
   * name is "x"
   * @see NewIndeterminates(const std::array<std::string, rows>& names)
   *
   * @exclude_from_pydrake_mkdoc{Overloads that require explicit template
   * arguments (rows) are not bound in pydrake.}
   */
  template <int rows>
  VectorIndeterminate<rows> NewIndeterminates(const std::string& name = "x") {
    std::array<std::string, rows> names;
    int offset = (name.compare("x") == 0) ? num_vars() : 0;
    for (int i = 0; i < rows; ++i) {
      names[i] = name + "(" + std::to_string(offset + i) + ")";
    }
    return NewIndeterminates<rows>(names);
  }

  /**
   * Adds indeterminates to this MathematicalProgram.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  VectorXIndeterminate NewIndeterminates(int rows,
                                         const std::vector<std::string>& names);

  /**
   * Adds indeterminates to this MathematicalProgram, with default name
   * "x".
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  VectorXIndeterminate NewIndeterminates(int rows,
                                         const std::string& name = "x");

  /**
   * Adds indeterminates, appending them to an internal vector of any
   * existing vars.
   * @param rows  The number of rows in the new indeterminates.
   * @param cols  The number of columns in the new indeterminates.
   * @param names A vector of strings containing the name for each variable.
   * @return The MatrixIndeterminate of size rows x cols, containing the
   * new vars (not all the vars stored).
   *
   * Example:
   * @code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewIndeterminates(2, 3, {"x1", "x2", "x3", "x4",
   * "x5", "x6"});
   * @endcode
   * This adds a 2 x 3 matrix indeterminates into the program.
   *
   * The name of the variable is only used for the user in order to ease
   * readability.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  MatrixXIndeterminate NewIndeterminates(int rows, int cols,
                                         const std::vector<std::string>& names);

  /**
   * Adds indeterminates to this MathematicalProgram, with default name
   * "X". The new variables are returned and viewed as a matrix, with size
   * @p rows x @p cols.
   * @see NewIndeterminates(int rows, int cols, const
   * std::vector<std::string>& names);
   */
  MatrixXIndeterminate NewIndeterminates(int rows, int cols,
                                         const std::string& name = "X");

  /** Adds indeterminate.
   * This method appends an indeterminate to the end of the program's old
   * indeterminates, if `new_indeterminate` is not already in the program's old
   * indeterminates.
   * @param new_indeterminate The indeterminate to be appended to the
   * program's old indeterminates.
   * @return indeterminate_index The index of the added indeterminate in the
   * program's indeterminates. i.e. prog.indeterminates()(indeterminate_index) =
   * new_indeterminate.
   * @pre `new_indeterminate` should not intersect with the program's
   * decision variables.
   * @pre new_indeterminate should be of CONTINUOUS type.
   */
  int AddIndeterminate(const symbolic::Variable& new_indeterminate);

  /** Adds indeterminates.
   * This method appends some indeterminates to the end of the program's old
   * indeterminates.
   * @param new_indeterminates The indeterminates to be appended to the
   * program's old indeterminates.
   * @pre `new_indeterminates` should not intersect with the program's old
   * decision variables.
   * @pre Each entry in new_indeterminates should be of CONTINUOUS type.
   */
  void AddIndeterminates(
      const Eigen::Ref<const MatrixXIndeterminate>& new_indeterminates);

  /** Adds indeterminates.
   * This method appends some indeterminates to the end of the program's old
   * indeterminates.
   * @param new_indeterminates The indeterminates to be appended to the
   * program's old indeterminates.
   * @pre `new_indeterminates` should not intersect with the program's old
   * decision variables.
   * @pre Each entry in new_indeterminates should be of CONTINUOUS type.
   */
  void AddIndeterminates(const symbolic::Variables& new_indeterminates);

  /**
   * Adds a callback method to visualize intermediate results of the
   * optimization.
   *
   * @note Just like other costs/constraints, not all solvers support callbacks.
   * Adding a callback here will force MathematicalProgram::Solve to select a
   * solver that support callbacks.  For instance, adding a visualization
   * callback to a quadratic programming problem may result in using a nonlinear
   * programming solver as the default solver.
   *
   * @param callback a std::function that accepts an Eigen::Vector of doubles
   * representing the bound decision variables.
   * @param vars the decision variables that should be passed to the callback.
   */
  Binding<VisualizationCallback> AddVisualizationCallback(
      const VisualizationCallback::CallbackFunction& callback,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a callback method to visualize intermediate results of the
   * optimization.
   *
   * @note Just like other costs/constraints, not all solvers support callbacks.
   * Adding a callback here will force MathematicalProgram::Solve to select a
   * solver that support callbacks.  For instance, adding a visualization
   * callback to a quadratic programming problem may result in using a nonlinear
   * programming solver as the default solver.
   *
   * @param callback a std::function that accepts an Eigen::Vector of doubles
   * representing the for the bound decision variables.
   * @param vars the decision variables that should be passed to the callback.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<VisualizationCallback> AddVisualizationCallback(
      const VisualizationCallback::CallbackFunction& callback,
      const VariableRefList& vars) {
    return AddVisualizationCallback(callback,
                                    ConcatenateVariableRefList((vars)));
  }

  /**
   * Adds a generic cost to the optimization program.
   *
   * @pydrake_mkdoc_identifier{1args_binding_cost}
   */
  Binding<Cost> AddCost(const Binding<Cost>& binding);

  /**
   * Adds a cost type to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   *
   * @pydrake_mkdoc_identifier{2args_obj_vars}
   */
  template <typename C>
  auto AddCost(const std::shared_ptr<C>& obj,
               const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    // Redirect to the appropriate type
    // Use auto to enable the overloading method to upcast if needed
    return AddCost(internal::CreateBinding(obj, vars));
  }

  /**
   * Adds a generic cost to the optimization program.
   * @param obj The added objective.
   * @param vars The decision variables on which the cost depend.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename C>
  auto AddCost(const std::shared_ptr<C>& obj, const VariableRefList& vars) {
    return AddCost(obj, ConcatenateVariableRefList(vars));
  }

  /**
   * Convert an input of type @p F to a FunctionCost object.
   * @tparam F This class should have functions numInputs(), numOutputs and
   * eval(x, y).
   */
  template <typename F>
  static std::shared_ptr<Cost> MakeCost(F&& f) {
    return MakeFunctionCost(f);
  }

  /**
   * Adds a cost to the optimization program on a list of variables.
   * @tparam F it should define functions numInputs, numOutputs and eval. Check
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename F>
    requires(internal::is_cost_functor_candidate<F>::value)
  Binding<Cost> AddCost(F&& f, const VariableRefList& vars) {
    return AddCost(f, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost to the optimization program on an Eigen::Vector containing
   * decision variables.
   * @tparam F Type that defines functions numInputs, numOutputs and eval.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename F>
    requires(internal::is_cost_functor_candidate<F>::value)
  Binding<Cost> AddCost(F&& f,
                        const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    auto c = MakeFunctionCost(std::forward<F>(f));
    return AddCost(c, vars);
  }

  /**
   * Statically assert if a user inadvertently passes a
   * binding-compatible Constraint.
   * @tparam F The type to check.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename F, typename Vars>
    requires(internal::assert_if_is_constraint<F>::value)
  Binding<Cost> AddCost(F&&, Vars&&) {
    throw std::runtime_error("This will assert at compile-time.");
  }

  /**
   * Adds a cost term of the form c'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearCost> AddCost(const Binding<LinearCost>& binding);

  /**
   * Adds a linear cost term of the form a'*x + b.
   * @param e A linear symbolic expression.
   * @pre e is a linear expression a'*x + b, where each entry of x is a decision
   * variable in the mathematical program.
   * @return The newly added linear constraint, together with the bound
   * variables.
   */
  Binding<LinearCost> AddLinearCost(const symbolic::Expression& e);

  /**
   * Adds a linear cost term of the form a'*x + b.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddLinearCost(const Eigen::Ref<const Eigen::VectorXd>& a,
                                    double b, const VariableRefList& vars) {
    return AddLinearCost(a, b, ConcatenateVariableRefList((vars)));
  }

  /**
   * Adds a linear cost term of the form a'*x + b.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  Binding<LinearCost> AddLinearCost(
      const Eigen::Ref<const Eigen::VectorXd>& a, double b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a linear cost term of the form a'*x.
   * Applied to a subset of the variables and pushes onto
   * the linear cost data structure.
   */
  template <typename VarType>
  Binding<LinearCost> AddLinearCost(const Eigen::Ref<const Eigen::VectorXd>& a,
                                    const VarType& vars) {
    const double b = 0.;
    return AddLinearCost(a, b, vars);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables and pushes onto
   * the quadratic cost data structure.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<QuadraticCost> AddCost(const Binding<QuadraticCost>& binding);

  /**
   * Add a quadratic cost term of the form 0.5*x'*Q*x + b'*x + c.
   * @param e A quadratic symbolic expression.
   * @param is_convex Whether the cost is already known to be convex. If
   * is_convex=nullopt (the default), then Drake will determine if `e` is a
   * convex quadratic cost or not. To improve the computation speed, the user
   * can set is_convex if the user knows whether the cost is convex or not.
   * @throws std::exception if the expression is not quadratic.
   * @return The newly added cost together with the bound variables.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const symbolic::Expression& e,
      std::optional<bool> is_convex = std::nullopt);

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x.
   * Applied to subset of the variables.
   * @param is_convex Whether the cost is already known to be convex. If
   * is_convex=nullopt (the default), then Drake will determine if this is a
   * convex quadratic cost or not (by checking if matrix Q is positive
   * semidefinite or not). To improve the computation speed, the user can set
   * is_convex if the user knows whether the cost is convex or not.
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars,
      std::optional<bool> is_convex = std::nullopt) {
    return AddQuadraticCost(Q, b, ConcatenateVariableRefList(vars), is_convex);
  }

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x + c
   * Applied to subset of the variables.
   * @param is_convex Whether the cost is already known to be convex. If
   * is_convex=nullopt (the default), then Drake will determine if this is a
   * convex quadratic cost or not. To improve the computation speed, the user
   * can set is_convex if the user knows whether the cost is convex or not.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, double c,
      const Eigen::Ref<const VectorXDecisionVariable>& vars,
      std::optional<bool> is_convex = std::nullopt);

  /**
   * Adds a cost term of the form 0.5*x'*Q*x + b'x
   * Applied to subset of the variables.
   * @param is_convex Whether the cost is already known to be convex. If
   * is_convex=nullopt (the default), then Drake will determine if this is a
   * convex quadratic cost or not. To improve the computation speed, the user
   * can set is_convex if the user knows whether the cost is convex or not.
   */
  Binding<QuadraticCost> AddQuadraticCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars,
      std::optional<bool> is_convex = std::nullopt);

  /**
   * Adds a cost term of the form w*|x-x_desired|^2.
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      double w, const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const VariableRefList& vars) {
    return AddQuadraticErrorCost(w, x_desired,
                                 ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a cost term of the form w*|x-x_desired|^2.
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      double w, const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const VariableRefList& vars);

  /**
   * Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).
   */
  Binding<QuadraticCost> AddQuadraticErrorCost(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& x_desired,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a quadratic cost of the form |Ax-b|²=(Ax-b)ᵀ(Ax-b)
   */
  Binding<QuadraticCost> Add2NormSquaredCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return Add2NormSquaredCost(A, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a quadratic cost of the form |Ax-b|²=(Ax-b)ᵀ(Ax-b)
   */
  Binding<QuadraticCost> Add2NormSquaredCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddCost(Make2NormSquaredCost(A, b), vars);
  }

  /**
   * Adds an L2 norm cost |Ax+b|₂ (notice this cost is not quadratic since we
   * don't take the square of the L2 norm).
   * Refer to AddL2NormCost for more details.
   */
  Binding<L2NormCost> AddCost(const Binding<L2NormCost>& binding);

  // TODO(hongkai.dai): support L2NormCost in each solver.
  /**
   * Adds an L2 norm cost |Ax+b|₂ (notice this cost is not quadratic since we
   * don't take the square of the L2 norm).
   * @note Currently kL2NormCost is supported by SnoptSolver, IpoptSolver,
   * NloptSolver, GurobiSolver, MosekSolver, ClarabelSolver, and SCSSolver.
   * @pydrake_mkdoc_identifier{3args_A_b_vars}
   */
  Binding<L2NormCost> AddL2NormCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds an L2 norm cost |Ax+b|₂ (notice this cost is not quadratic since we
   * don't take the square of the L2 norm)
   * @pydrake_mkdoc_identifier{3args_A_b_vars_list}
   */
  Binding<L2NormCost> AddL2NormCost(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                    const Eigen::Ref<const Eigen::VectorXd>& b,
                                    const VariableRefList& vars) {
    return AddL2NormCost(A, b, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds an L2 norm cost |Ax+b|₂ from a symbolic expression which can be
   * decomposed into sqrt((Ax+b)'(Ax+b)). See
   * symbolic::DecomposeL2NormExpression for details on the tolerance
   * parameters.
   * @throws std::exception if @p e cannot be decomposed into an L2 norm.
   * @pydrake_mkdoc_identifier{expression}
   */
  Binding<L2NormCost> AddL2NormCost(const symbolic::Expression& e,
                                    double psd_tol = 1e-8,
                                    double coefficient_tol = 1e-8);

  // TODO(hongkai.dai) Decide whether to deprecate this.
  /**
   * Adds an L2 norm cost min |Ax+b|₂ as a linear cost min s
   * on the slack variable s, together with a Lorentz cone constraint
   * s ≥ |Ax+b|₂
   * Many conic optimization solvers (Gurobi, MOSEK™, SCS, etc) natively prefers
   * this form of linear cost + conic constraints. So if you are going to use
   * one of these conic solvers, then add the L2 norm cost using this function
   * instead of AddL2NormCost().
   * @return (s, linear_cost, lorentz_cone_constraint). `s` is the slack
   * variable (with variable name string as "slack"), `linear_cost` is the cost
   * on `s`, and `lorentz_cone_constraint` is the constraint s≥|Ax+b|₂
   */
  auto AddL2NormCostUsingConicConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars)
      -> std::tuple<symbolic::Variable, Binding<LinearCost>,
                    Binding<LorentzConeConstraint>>;

  /**
   * Adds an L1 norm cost min |Ax+b|₁ as a linear cost min Σᵢsᵢ on the slack
   * variables sᵢ, together with the constraints (for each i) sᵢ ≥ (|Ax+b|)ᵢ,
   * which itself is written sᵢ ≥ (Ax+b)ᵢ and sᵢ ≥ -(Ax+b)ᵢ.
   * @return (s, linear_cost, linear_constraint). `s` is the vector of slack
   * variables, `linear_cost` is the cost on `s`, and `linear_constraint` is the
   * constraint encoding s ≥ Ax+b and s ≥ -(Ax+b).
   */
  auto AddL1NormCostInEpigraphForm(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars)
      -> std::tuple<VectorX<symbolic::Variable>, Binding<LinearCost>,
                    Binding<LinearConstraint>>;

  /**
   * Adds a cost term in the polynomial form.
   * @param e A symbolic expression in the polynomial form.
   * @return The newly created cost and the bound variables.
   */
  Binding<PolynomialCost> AddPolynomialCost(const symbolic::Expression& e);

  /**
   * Adds a cost in the symbolic form.
   * @return The newly created cost, together with the bound variables.
   */
  Binding<Cost> AddCost(const symbolic::Expression& e);

  /**
   * @anchor log_determinant
   * @name Matrix log determinant
   * Represents the log-determinant of `X` by introducing slack variables t, and
   * a lower triangular matrix Z and imposing the constraints
   *
   *    ⌈X         Z⌉ is positive semidifinite.
   *    ⌊Zᵀ  diag(Z)⌋
   *
   *    log(Z(i, i)) >= t(i)
   *
   * Since log(det(X)) is a concave function of X, we can either lower bound
   * it's value by imposing the constraint `∑ᵢt(i) >= lower` or maximize its
   * value by adding the cost -∑ᵢt(i) using convex optimization.
   * @note The constraint log(Z(i, i)) >= t(i) is imposed as an exponential cone
   * constraint. Please make sure your have a solver that supports exponential
   * cone constraint (currently SCS and Mosek do).
   * @note The constraint that
   *
   *     ⌈X         Z⌉ is positive semidifinite.
   *     ⌊Zᵀ  diag(Z)⌋
   *
   * already implies that X is positive semidefinite. The user DO NOT need to
   * separately impose the constraint that X being psd.
   *
   * Refer to
   * https://docs.mosek.com/modeling-cookbook/sdo.html#log-determinant for more
   * details.
   */
  //@{
  /**
   * Maximize the log determinant. See @ref log_determinant for more details.
   * @param X A symmetric positive semidefinite matrix X, whose log(det(X)) will
   * be maximized.
   * @return (cost, t, Z) cost is -∑ᵢt(i), we also return the newly created
   * slack variables t and the lower triangular matrix Z. Note that Z is not a
   * matrix of symbolic::Variable but symbolic::Expression, because the
   * upper-diagonal entries of Z are not variable, but expression 0.
   * @pre X is a symmetric matrix.
   */
  // TODO(hongkai.dai): return the lower-triangular of Z as
  // VectorX<symbolic::Variable>.
  auto AddMaximizeLogDeterminantCost(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X)
      -> std::tuple<Binding<LinearCost>, VectorX<symbolic::Variable>,
                    MatrixX<symbolic::Expression>>;

  /**
   * Impose the constraint log(det(X)) >= lower. See @ref log_determinant for
   * more details.
   * @param X A symmetric positive semidefinite matrix X.
   * @param lower The lower bound of log(det(X))
   * @return (constraint, t, Z) constraint is ∑ᵢt(i) >= lower, we also return
   * the newly created slack variables t and the lower triangular matrix Z. Note
   * that Z is not a matrix of symbolic::Variable but symbolic::Expression,
   * because the upper-diagonal entries of Z are not variable, but expression 0.
   * @pre X is a symmetric matrix.
   */
  // TODO(hongkai.dai): return the lower-triangular of Z as
  // VectorX<symbolic::Variable>.
  auto AddLogDeterminantLowerBoundConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X, double lower)
      -> std::tuple<Binding<LinearConstraint>, VectorX<symbolic::Variable>,
                    MatrixX<symbolic::Expression>>;
  //@}

  /**
   * @anchor maximize_geometric_mean
   * @name    Maximize geometric mean
   * Adds the cost to maximize the geometric mean of z = Ax+b, i.e.
   * power(∏ᵢz(i), 1/n), where z ∈ ℝⁿ, z(i) >= 0. Mathematically, the cost we
   * add is -power(∏ᵢz(i), 1/r), where r = power(2, ceil(log₂n)), namely r is
   * the smallest power of 2 that is no smaller than the size of z. For example,
   * if z ∈ ℝ², then the added cost is -power(z(0)*z(1), 1/2) if z ∈ ℝ³, then
   * the added cost is -power(z(0)*z(1)*z(2), 1/4).
   *
   * In order to add this cost, we need to introduce a set of second-order cone
   * constraints. For example, to maximize power(z(0) * z(1), 1/2), we
   * introduce the slack variable w(0), together with the second order cone
   * constraint w(0)² ≤ z(0) * z(1), z(0) ≥ 0, z(1) ≥ 0, and we maximize w(0).
   *
   * To maximize power(z(0) * z(1) * z(2), 1/ 4), we introduce the slack
   * variable w(0), w(1), w(2), together with the second order cone constraints
   * <pre>
   * w(0)² ≤ z(0) * z(1), z(0) ≥ 0, z(1) ≥ 0
   * w(1)² ≤ z(2), z(2) ≥ 0
   * w(2)² ≤ w(0) * w(1), w(0) ≥ 0, w(1) ≥ 0
   * </pre>
   * and we maximize w(2).
   */
  //@{
  /**
   * An overloaded version of @ref maximize_geometric_mean.
   * @return cost The added cost (note that since MathematicalProgram only
   * minimizes the cost, the returned cost evaluates to -power(∏ᵢz(i), 1/n)
   * where z = A*x+b.
   * @pre A.rows() == b.rows(), A.rows() >= 2.
   */
  Binding<LinearCost> AddMaximizeGeometricMeanCost(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x);

  /**
   * An overloaded version of @ref maximize_geometric_mean.
   * We add the cost to maximize the geometric mean of x, i.e., c*power(∏ᵢx(i),
   * 1/n).
   * @param c The positive coefficient of the geometric mean cost, @default
   * is 1.
   * @return cost The added cost (note that since MathematicalProgram only
   * minimizes the cost, the returned cost evaluates to -c * power(∏ᵢx(i), 1/n).
   * @pre x.rows() >= 2.
   * @pre c > 0.
   */
  Binding<LinearCost> AddMaximizeGeometricMeanCost(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& x, double c = 1.0);
  //@}

  /**
   * Adds a generic constraint to the program. This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   *
   * @note If @p binding.evaluator()->num_constraints() == 0, then this
   * constraint is not added into the MathematicalProgram. We return @p binding
   * directly.
   */
  Binding<Constraint> AddConstraint(const Binding<Constraint>& binding);

  /**
   * Adds one row of constraint lb <= e <= ub where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   * 2. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
   *
   * @param e A symbolic expression of the decision variables.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   *
   * The resulting constraint may be a BoundingBoxConstraint, LinearConstraint,
   * LinearEqualityConstraint, QuadraticConstraint, or ExpressionConstraint,
   * depending on the arguments.  Constraints of the form x == 1 (which could
   * be created as a BoundingBoxConstraint or LinearEqualityConstraint) will be
   * constructed as a LinearEqualityConstraint.
   */
  Binding<Constraint> AddConstraint(const symbolic::Expression& e, double lb,
                                    double ub);

  /**
   * Adds constraints represented by symbolic expressions to the program. It
   * throws if <tt>lb <= v <= ub</tt> includes trivial/unsatisfiable
   * constraints.
   *
   * @overload Binding<Constraint> AddConstraint(const symbolic::Expression& e,
   *    double lb, double ub)
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<Constraint> AddConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& v,
      const Eigen::Ref<const Eigen::MatrixXd>& lb,
      const Eigen::Ref<const Eigen::MatrixXd>& ub);

  /**
   * Add a constraint represented by a symbolic formula to the program. The
   * input formula @p f can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   * 4. A conjunction of relational formulas where each conjunct is
   *    a relational formula matched by 1, 2, or 3.
   *
   * Note that first two cases might return an object of
   * Binding<BoundingBoxConstraint>, Binding<LinearConstraint>, or
   * Binding<ExpressionConstraint>, depending
   * on @p f. Also the third case might return an object of
   * Binding<LinearEqualityConstraint> or Binding<ExpressionConstraint>.
   *
   * It throws an exception if
   *  1. @p f is not matched with one of the above patterns. Especially, strict
   *     inequalities (<, >) are not allowed.
   *  2. @p f is either a trivial constraint such as "1 <= 2" or an
   *     unsatisfiable constraint such as "2 <= 1".
   *  3. It is not possible to find numerical bounds of `e1` and `e2` where @p f
   *     = e1 ≃ e2. We allow `e1` and `e2` to be infinite but only if there are
   *     no other terms. For example, `x <= ∞` is allowed. However, `x - ∞ <= 0`
   *     is not allowed because `x ↦ ∞` introduces `nan` in the evaluation.
   */
  Binding<Constraint> AddConstraint(const symbolic::Formula& f);

  /**
   * Adds a constraint represented by an Eigen::Matrix<symbolic::Formula> or
   * Eigen::Array<symbolic::Formula> to the program. A common use-case of this
   * function is to add a constraint with the element-wise comparison between
   * two Eigen matrices, using `A.array() <= B.array()`. See the following
   * example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A = ...;
   *   Eigen::Vector2d b = ...;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   prog.AddConstraint((A * x).array() <= b.array());
   * @endcode
   *
   * A formula in @p formulas can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   *
   * It throws an exception if AddConstraint(const symbolic::Formula& f)
   * throws an exception for f ∈ @p formulas.
   *
   * @overload Binding<Constraint> AddConstraint(const symbolic::Formula& f)
   *
   * @tparam Derived Eigen::Matrix or Eigen::Array with Formula as the Scalar.
   */
  template <typename Derived>
    requires(is_eigen_scalar_same<Derived, symbolic::Formula>::value)
  Binding<Constraint> AddConstraint(const Eigen::DenseBase<Derived>& formulas) {
    return AddConstraint(internal::ParseConstraint(formulas));
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con, const VariableRefList& vars) {
    return AddConstraint(con, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a generic constraint to the program.  This should
   * only be used if a more specific type of constraint is not
   * available, as it may require the use of a significantly more
   * expensive solver.
   * @pydrake_mkdoc_identifier{2args_con_vars}
   */
  template <typename C>
  auto AddConstraint(std::shared_ptr<C> con,
                     const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddConstraint(internal::CreateBinding(con, vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearConstraint> AddConstraint(
      const Binding<LinearConstraint>& binding);

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(A, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds sparse linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(A, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   *
   * @pydrake_mkdoc_identifier{4args_A_lb_ub_dense}
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds sparse linear constraints referencing potentially a subset
   * of the decision variables (defined in the vars parameter).
   *
   * @pydrake_mkdoc_identifier{4args_A_lb_ub_sparse}
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::SparseMatrix<double>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars The decision variables on which to impose the linear
   * constraint.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double lb, double ub,
      const VariableRefList& vars) {
    return AddLinearConstraint(a, lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds one row of linear constraint referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * lb <= a*vars <= ub
   * @param a A row vector.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   * @param vars The decision variables on which to impose the linear
   * constraint.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double lb, double ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddLinearConstraint(a, Vector1d(lb), Vector1d(ub), vars);
  }

  /**
   * Adds one row of linear constraint lb <= e <= ub where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. @p e is a non-linear expression.
   * 2. <tt>lb <= e <= ub</tt> is a trivial constraint such as 1 <= 2 <= 3.
   * 3. <tt>lb <= e <= ub</tt> is unsatisfiable such as 1 <= -5 <= 3
   *
   * @param e A linear symbolic expression in the form of <tt>c0 + c1 * v1 +
   * ... + cn * vn</tt> where @c c_i is a constant and @v_i is a variable.
   * @param lb A scalar, the lower bound.
   * @param ub A scalar, the upper bound.
   */
  Binding<LinearConstraint> AddLinearConstraint(const symbolic::Expression& e,
                                                double lb, double ub);

  /**
   * Adds linear constraints represented by symbolic expressions to the
   * program. It throws if @v includes a non-linear expression or <tt>lb <= v <=
   * ub</tt> includes trivial/unsatisfiable constraints.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& v,
      const Eigen::Ref<const Eigen::MatrixXd>& lb,
      const Eigen::Ref<const Eigen::MatrixXd>& ub);

  /**
   * Add a linear constraint represented by a symbolic formula to the
   * program. The input formula @p f can be of the following forms:
   *
   * 1. e1 <= e2
   * 2. e1 >= e2
   * 3. e1 == e2
   * 4. A conjunction of relational formulas where each conjunct is
   *    a relational formula matched by 1, 2, or 3.
   *
   * Note that first two cases might return an object of
   * Binding<BoundingBoxConstraint> depending on @p f. Also the third case
   * returns an object of Binding<LinearEqualityConstraint>.
   *
   * It throws an exception if
   *  1. @p f is not matched with one of the above patterns. Especially, strict
   *     inequalities (<, >) are not allowed.
   *  2. @p f includes a non-linear expression.
   *  3. @p f is either a trivial constraint such as "1 <= 2" or an
   *     unsatisfiable constraint such as "2 <= 1".
   *  4. It is not possible to find numerical bounds of `e1` and `e2` where @p f
   *     = e1 ≃ e2. We allow `e1` and `e2` to be infinite but only if there are
   *     no other terms. For example, `x <= ∞` is allowed. However, `x - ∞ <= 0`
   *     is not allowed because `x ↦ ∞` introduces `nan` in the evaluation.
   */
  Binding<LinearConstraint> AddLinearConstraint(const symbolic::Formula& f);

  /**
   * Add a linear constraint represented by an Eigen::Array<symbolic::Formula>
   * to the program. A common use-case of this function is to add a linear
   * constraint with the element-wise comparison between two Eigen matrices,
   * using `A.array() <= B.array()`. See the following example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   Eigen::Vector2d b;
   *   ... // set up A and b
   *   prog.AddLinearConstraint((A * x).array() <= b.array());
   * @endcode
   *
   * A formula in @p formulas can be of the following forms:
   *
   *  1. e1 <= e2
   *  2. e1 >= e2
   *  3. e1 == e2
   *
   * It throws an exception if AddLinearConstraint(const symbolic::Formula& f)
   * throws an exception for f ∈ @p formulas.
   * @tparam Derived An Eigen Array type of Formula.
   */
  Binding<LinearConstraint> AddLinearConstraint(
      const Eigen::Ref<const Eigen::Array<symbolic::Formula, Eigen::Dynamic,
                                          Eigen::Dynamic>>& formulas);

  /**
   * Adds linear equality constraints referencing potentially a
   * subset of the decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddConstraint(
      const Binding<LinearEqualityConstraint>& binding);

  /**
   * Adds one row of linear constraint e = b where @p e is a symbolic
   * expression.
   * @throws std::exception if
   * 1. @p e is a non-linear expression.
   * 2. @p e is a constant.
   *
   * @param e A linear symbolic expression in the form of <tt>c0 + c1 * x1 +
   * ... + cn * xn</tt> where @c c_i is a constant and @x_i is a variable.
   * @param b A scalar.
   * @return The newly added linear equality constraint, together with the
   * bound variable.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Expression& e, double b);

  /**
   * Adds a linear equality constraint represented by a symbolic formula to the
   * program. The input formula @p f is either an equality formula (`e1 == e2`)
   * or a conjunction of equality formulas.
   *
   * It throws an exception if
   *
   * 1. @p f is neither an equality formula nor a conjunction of equalities.
   * 2. @p f includes a non-linear expression.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const symbolic::Formula& f);

  /**
   * Adds a linear equality constraint represented by an
   * Eigen::Array<symbolic::Formula> to the program. A common use-case of this
   * function is to add a linear constraint with the element-wise comparison
   * between two Eigen matrices, using `A.array() == B.array()`. See the
   * following example.
   *
   * @code
   *   MathematicalProgram prog;
   *   Eigen::Matrix<double, 2, 2> A;
   *   auto x = prog.NewContinuousVariables(2, "x");
   *   Eigen::Vector2d b;
   *   ... // set up A and b
   *   prog.AddLinearConstraint((A * x).array() == b.array());
   * @endcode
   *
   * It throws an exception if AddLinearConstraint(const symbolic::Formula& f)
   * throws an exception for f ∈ `formulas`.
   * @tparam Derived An Eigen Array type of Formula. */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::Array<symbolic::Formula, Eigen::Dynamic,
                                          Eigen::Dynamic>>& formulas);

  /**
   * Adds linear equality constraints \f$ v = b \f$, where \p v(i) is a symbolic
   * linear expression.
   * @throws std::exception if
   * 1. @p v(i) is a non-linear expression.
   * 2. @p v(i) is a constant.
   *
   * @tparam DerivedV An Eigen Matrix type of Expression. A column vector.
   * @tparam DerivedB An Eigen Matrix type of double. A column vector.
   * @param v v(i) is a linear symbolic expression in the form of
   * <tt> c0 + c1 * x1 + ... + cn * xn </tt> where ci is a constant and @xi is
   * a variable.
   * @param b A vector of doubles.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   */
  template <typename DerivedV, typename DerivedB>
    requires(is_eigen_vector_expression_double_pair<DerivedV, DerivedB>::value)
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedV>& v,
      const Eigen::MatrixBase<DerivedB>& b) {
    return AddConstraint(internal::ParseLinearEqualityConstraint(v, b));
  }

  /**
   * Adds a linear equality constraint for a matrix of linear expression @p V,
   * such that V(i, j) = B(i, j). If V is a symmetric matrix, then the user
   * may only want to constrain the lower triangular part of V.
   * This function is meant to provide convenience to the user, it incurs
   * additional copy and memory allocation. For faster speed, add each column
   * of the matrix equality in a for loop.
   * @tparam DerivedV An Eigen Matrix type of Expression. The number of columns
   * at compile time should not be 1.
   * @tparam DerivedB An Eigen Matrix type of double.
   * @param V An Eigen Matrix of symbolic expressions. V(i, j) should be a
   * linear expression.
   * @param B An Eigen Matrix of doubles.
   * @param lower_triangle If true, then only the lower triangular part of @p V
   * is constrained, otherwise the whole matrix V is constrained. @default is
   * false.
   * @return The newly added linear equality constraint, together with the
   * bound variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename DerivedV, typename DerivedB>
    requires(
        is_eigen_nonvector_expression_double_pair<DerivedV, DerivedB>::value)
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::MatrixBase<DerivedV>& V,
      const Eigen::MatrixBase<DerivedB>& B, bool lower_triangle = false) {
    return AddConstraint(
        internal::ParseLinearEqualityConstraint(V, B, lower_triangle));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousVariables(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   prog.AddLinearEqualityConstraint(Aeq, beq, {x.segment<1>(2),
   *                                    x.segment<1>(5)});
   * @endcode
   * The code above imposes constraints
   * @f[-x(2) + 2x(5) = 1 @f]
   * @f[ x(2) +  x(5) = 3 @f]
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const VariableRefList& vars) {
    return AddLinearEqualityConstraint(Aeq, beq,
                                       ConcatenateVariableRefList(vars));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables using a sparse A matrix.
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::SparseMatrix<double>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const VariableRefList& vars) {
    return AddLinearEqualityConstraint(Aeq, beq,
                                       ConcatenateVariableRefList(vars));
  }

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables.
   *
   * Example: to add two equality constraints which only depend on two of the
   * elements of x, you could use
   * @code{.cc}
   *   auto x = prog.NewContinuousVariables(6,"myvar");
   *   Eigen::Matrix2d Aeq;
   *   Aeq << -1, 2,
   *           1, 1;
   *   Eigen::Vector2d beq(1, 3);
   *   // Imposes constraint
   *   // -x(0) + 2x(1) = 1
   *   //  x(0) +  x(1) = 3
   *   prog.AddLinearEqualityConstraint(Aeq, beq, x.head<2>());
   * @endcode
   *
   * @pydrake_mkdoc_identifier{3args_Aeq_beq_dense}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /** AddLinearEqualityConstraint
   *
   * Adds linear equality constraints referencing potentially a subset of
   * the decision variables using a sparse A matrix.
   * @pydrake_mkdoc_identifier{3args_Aeq_beq_sparse}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::SparseMatrix<double>& Aeq,
      const Eigen::Ref<const Eigen::VectorXd>& beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds one row of linear equality constraint referencing potentially a subset
   * of decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   * @param vars The decision variables on which the constraint is imposed.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq,
      const VariableRefList& vars) {
    return AddLinearEqualityConstraint(a, beq,
                                       ConcatenateVariableRefList(vars));
  }

  /**
   * Adds one row of linear equality constraint referencing potentially a subset
   * of decision variables.
   * @f[
   * ax = beq
   * @f]
   * @param a A row vector.
   * @param beq A scalar.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<LinearEqualityConstraint> AddLinearEqualityConstraint(
      const Eigen::Ref<const Eigen::RowVectorXd>& a, double beq,
      const Eigen::Ref<const VectorXDecisionVariable>& vars) {
    return AddLinearEqualityConstraint(a, Vector1d(beq), vars);
  }

  /**
   * Adds bounding box constraints referencing potentially a subest of the
   * decision variables.
   * @param binding Binds a BoundingBoxConstraint with some decision variables,
   * such that
   * binding.evaluator()->lower_bound()(i) <= binding.variables()(i)
   *                   <= binding.evaluator().upper_bound()(i)
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<BoundingBoxConstraint> AddConstraint(
      const Binding<BoundingBoxConstraint>& binding);

  /** AddBoundingBoxConstraint
   *
   * Adds bounding box constraints referencing potentially a
   * subset of the decision variables (defined in the vars parameter).
   * Example
   * \code{.cc}
   * MathematicalProgram prog;
   * auto x = prog.NewContinuousVariables<2>("x");
   * auto y = prog.NewContinuousVariables<1>("y");
   * Eigen::Vector3d lb(0, 1, 2);
   * Eigen::Vector3d ub(1, 2, 3);
   * // Imposes the constraint
   * // 0 ≤ x(0) ≤ 1
   * // 1 ≤ x(1) ≤ 2
   * // 2 ≤ y    ≤ 3
   * prog.AddBoundingBoxConstraint(lb, ub, {x, y});
   * \endcode
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      const VariableRefList& vars) {
    return AddBoundingBoxConstraint(lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds bounding box constraints referencing potentially a subset of the
   * decision variables.
   * @param lb The lower bound.
   * @param ub The upper bound.
   * @param vars Will imposes constraint lb(i, j) <= vars(i, j) <= ub(i, j).
   * @return The newly constructed BoundingBoxConstraint.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& lb,
      const Eigen::Ref<const Eigen::MatrixXd>& ub,
      const Eigen::Ref<const MatrixXDecisionVariable>& vars);

  /**
   * Adds bounds for a single variable.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param var The decision variable.
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const symbolic::Variable& var) {
    MatrixDecisionVariable<1, 1> var_matrix(var);
    return AddBoundingBoxConstraint(Vector1d(lb), Vector1d(ub), var_matrix);
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in the list.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const VariableRefList& vars) {
    return AddBoundingBoxConstraint(lb, ub, ConcatenateVariableRefList(vars));
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in @p vars.
   * @tparam Derived An Eigen Vector type with Variable as the scalar
   * type.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <typename Derived>
    requires(std::is_same_v<typename Derived::Scalar, symbolic::Variable> &&
             Derived::ColsAtCompileTime == 1)
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const Eigen::MatrixBase<Derived>& vars) {
    const int kSize = Derived::RowsAtCompileTime;
    return AddBoundingBoxConstraint(
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), lb),
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), ub), vars);
  }

  /**
   * Adds the same scalar lower and upper bound to every variable in @p vars.
   * @tparam Derived An Eigen::Matrix with Variable as the scalar
   * type. The matrix has unknown number of columns at compile time, or has
   * more than one column.
   * @param lb Lower bound.
   * @param ub Upper bound.
   * @param vars The decision variables.
   */
  template <typename Derived>
    requires(std::is_same_v<typename Derived::Scalar, symbolic::Variable> &&
             Derived::ColsAtCompileTime != 1)
  Binding<BoundingBoxConstraint> AddBoundingBoxConstraint(
      double lb, double ub, const Eigen::MatrixBase<Derived>& vars) {
    const int kSize =
        Derived::RowsAtCompileTime != Eigen::Dynamic &&
                Derived::ColsAtCompileTime != Eigen::Dynamic
            ? Derived::RowsAtCompileTime * Derived::ColsAtCompileTime
            : Eigen::Dynamic;
    Eigen::Matrix<symbolic::Variable, kSize, 1> flat_vars(vars.size());
    for (int j = 0; j < vars.cols(); ++j) {
      for (int i = 0; i < vars.rows(); ++i) {
        flat_vars(j * vars.rows() + i) = vars(i, j);
      }
    }
    return AddBoundingBoxConstraint(
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), lb),
        Eigen::Matrix<double, kSize, 1>::Constant(vars.size(), ub), flat_vars);
  }

  /** Adds quadratic constraint.
   The quadratic constraint is of the form
   lb ≤ .5 xᵀQx + bᵀx ≤ ub
   where `x` might be a subset of the decision variables in this
   MathematicalProgram.
   Notice that if your quadratic constraint is convex, and you intend to solve
   the problem with a convex solver (like Mosek), then it is better to
   reformulate it with a second order cone constraint. See
   https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation for
   an explanation.
   @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<QuadraticConstraint> AddConstraint(
      const Binding<QuadraticConstraint>& binding);

  /** Adds quadratic constraint
   lb ≤ .5 xᵀQx + bᵀx ≤ ub
   Notice that if your quadratic constraint is convex, and you intend to solve
   the problem with a convex solver (like Mosek), then it is better to
   reformulate it with a second order cone constraint. See
   https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation for
   an explanation.
   @param vars x in the documentation above.
   @param hessian_type Whether the Hessian is positive semidefinite, negative
   semidefinite or indefinite. Drake will check the type if
   hessian_type=std::nullopt. Specifying the hessian type will speed this
   method up.
   @pre hessian_type should be correct if it is not std::nullopt, as we will
   blindly trust it in the downstream code.
   */
  Binding<QuadraticConstraint> AddQuadraticConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, double lb, double ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars,
      std::optional<QuadraticConstraint::HessianType> hessian_type =
          std::nullopt);

  /** Adds quadratic constraint
   lb ≤ .5 xᵀQx + bᵀx ≤ ub
   Notice that if your quadratic constraint is convex, and you intend to solve
   the problem with a convex solver (like Mosek), then it is better to
   reformulate it with a second order cone constraint. See
   https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation for
   an explanation.
   @param vars x in the documentation above.
   @param hessian_type Whether the Hessian is positive semidefinite, negative
   semidefinite or indefinite. Drake will check the type if
   hessian_type=std::nullopt. Specifying the hessian type will speed this
   method up.
   @pre hessian_type should be correct if it is not std::nullopt, as we will
   blindly trust it in the downstream code.
   */
  Binding<QuadraticConstraint> AddQuadraticConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, double lb, double ub,
      const VariableRefList& vars,
      std::optional<QuadraticConstraint::HessianType> hessian_type =
          std::nullopt);

  /** Overloads AddQuadraticConstraint, impose lb <= e <= ub where `e` is a
   quadratic expression.
   Notice that if your quadratic constraint is convex, and you intend to solve
   the problem with a convex solver (like Mosek), then it is better to
   reformulate it with a second order cone constraint. See
   https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation for
   an explanation.
   */
  Binding<QuadraticConstraint> AddQuadraticConstraint(
      const symbolic::Expression& e, double lb, double ub,
      std::optional<QuadraticConstraint::HessianType> hessian_type =
          std::nullopt);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset
   * of the decision variables.
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)² + ... + z(n-1)²}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddConstraint(
      const Binding<LorentzConeConstraint>& binding);

  /**
   * Adds a Lorentz cone constraint of the form Ax+b >= |Cx+d|₂ from a symbolic
   * formula with one side which can be decomposed into sqrt((Cx+d)'(Cx+d)).
   *
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   *
   * See symbolic::DecomposeL2NormExpression for details on the tolerance
   * parameters, @p psd_tol and @p coefficient_tol. Consider using the overload
   * which takes a vector of expressions to avoid the numerical decomposition.
   *
   * @throws std::exception if @p f cannot be decomposed into a Lorentz cone.
   * @pydrake_mkdoc_identifier{formula}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const symbolic::Formula& f,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth,
      double psd_tol = 1e-8, double coefficient_tol = 1e-8);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables.
   * @param v An Eigen::Vector of symbolic::Expression. Constraining that
   * \f[
   * v_0 \ge \sqrt{v_1^2 + ... + v_{n-1}^2}
   * \f]
   * @return The newly constructed Lorentz cone constraint with the bounded
   * variables.
   * For example, to add the Lorentz cone constraint
   *
   *     x+1 >= sqrt(y² + 2y + x² + 5),
   *          = sqrt((y+1)²+x²+2²)
   * The user could call
   * @code{cc}
   * Vector4<symbolic::Expression> v(x+1, y+1, x, 2.);
   * prog.AddLorentzConeConstraint(v);
   * @endcode
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth);

  /**
   * Adds Lorentz cone constraint on the linear expression v1 and quadratic
   * expression v2, such that v1 >= sqrt(v2)
   * @param linear_expression The linear expression v1.
   * @param quadratic_expression  The quadratic expression v2.
   * @param tol The tolerance to determine if the matrix in v2 is positive
   * semidefinite or not. @see DecomposePositiveQuadraticForm for more
   * explanation. @default is 0.
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   * @retval binding The newly added Lorentz cone constraint, together with the
   * bound variables.
   * @pre
   * 1. `v1` is a linear expression, in the form of c'*x + d.
   * 2. `v2` is a quadratic expression, in the form of
   *    <pre>
   *          x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * @throws std::exception if the preconditions are not satisfied.
   *
   * Notice this constraint is equivalent to the vector [z;y] is within a
   * Lorentz cone, where
   * <pre>
   *  z = v1
   *  y = R * x + d
   * </pre>
   * while (R, d) satisfies y'*y = x'*Q*x + b'*x + a
   * For example, to add the Lorentz cone constraint
   *
   *     x+1 >= sqrt(y² + 2y + x² + 4),
   * the user could call
   * @code{cc}
   * prog.AddLorentzConeConstraint(x+1, pow(y, 2) + 2 * y + pow(x, 2) + 4);
   * @endcode
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const symbolic::Expression& linear_expression,
      const symbolic::Expression& quadratic_expression, double tol = 0,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth);

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)² + ... + z(n-1)²}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The list of @f$ m @f$ decision variables.
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth) {
    return AddLorentzConeConstraint(A, b, ConcatenateVariableRefList(vars),
                                    eval_type);
  }

  /**
   * Adds Lorentz cone constraint referencing potentially a subset of the
   * decision variables (defined in the vars parameter).
   * The linear expression @f$ z=Ax+b @f$ is in the Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the Lorentz cone, if
   * <!--
   * z(0) >= sqrt{z(1)² + ... + z(n-1)²}
   * -->
   * @f[
   * z_0 \ge \sqrt{z_1^2 + ... + z_{n-1}^2}
   * @f]
   * @param A A @f$\mathbb{R}^{n\times m}@f$ matrix, whose number of columns
   * equals to the size of the decision variables.
   * @param b A @f$\mathbb{R}^n@f$ vector, whose number of rows equals to the
   * size of the decision variables.
   * @param vars The Eigen vector of @f$ m @f$ decision variables.
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   * @return The newly added Lorentz cone constraint.
   *
   * For example, to add the Lorentz cone constraint
   *
   *     x+1 >= sqrt(y² + 2y + x² + 5) = sqrt((y+1)² + x² + 2²),
   * the user could call
   * @code{cc}
   * Eigen::Matrix<double, 4, 2> A;
   * Eigen::Vector4d b;
   * A << 1, 0, 0, 1, 1, 0, 0, 0;
   * b << 1, 1, 0, 2;
   * // A * [x;y] + b = [x+1; y+1; x; 2]
   * prog.AddLorentzConeConstraint(A, b, Vector2<symbolic::Variable>(x, y));
   * @endcode
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth);

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)² + ... + x(m-1)²)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const VariableRefList& vars,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth) {
    return AddLorentzConeConstraint(ConcatenateVariableRefList(vars),
                                    eval_type);
  }

  /**
   * Imposes that a vector @f$ x\in\mathbb{R}^m @f$ lies in Lorentz cone. Namely
   * @f[
   * x_0 \ge \sqrt{x_1^2 + .. + x_{m-1}^2}
   * @f]
   * <!-->
   * x(0) >= sqrt(x(1)² + ... + x(m-1)²)
   * <-->
   * @param vars The stacked column of vars should lie within the Lorentz cone.
   * @param eval_type The evaluation type when evaluating the lorentz cone
   * constraint in generic optimization. Refer to
   * LorentzConeConstraint::EvalType for more details.
   * @return The newly added Lorentz cone constraint.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  template <int rows>
  Binding<LorentzConeConstraint> AddLorentzConeConstraint(
      const Eigen::MatrixBase<VectorDecisionVariable<rows>>& vars,
      LorentzConeConstraint::EvalType eval_type =
          LorentzConeConstraint::EvalType::kConvexSmooth) {
    Eigen::Matrix<double, rows, rows> A(vars.rows(), vars.rows());
    A.setIdentity();
    Eigen::Matrix<double, rows, 1> b(vars.rows());
    b.setZero();
    return AddLorentzConeConstraint(A, b, vars, eval_type);
  }

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables. The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)² + ... + z(n-1)²
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<RotatedLorentzConeConstraint> AddConstraint(
      const Binding<RotatedLorentzConeConstraint>& binding);

  /**
   * Adds rotated Lorentz cone constraint on the linear expression v1, v2 and
   * quadratic expression u, such that v1 * v2 >= u, v1 >= 0, v2 >= 0
   * @param linear_expression1 The linear expression v1.
   * @param linear_expression2 The linear expression v2.
   * @param quadratic_expression The quadratic expression u.
   * @param tol The tolerance to determine if the matrix in v2 is positive
   * semidefinite or not. @see DecomposePositiveQuadraticForm for more
   * explanation. @default is 0.
   * @retval binding The newly added rotated Lorentz cone constraint, together
   * with the bound variables.
   * @pre
   * 1. `linear_expression1` is a linear (affine) expression, in the form of
   *    v1 = c1'*x + d1.
   * 2. `linear_expression2` is a linear (affine) expression, in the form of
   *    v2 = c2'*x + d2.
   * 2. `quadratic_expression` is a quadratic expression, in the form of
   *    <pre>
   *          u = x'*Q*x + b'x + a
   *    </pre>
   *    Also the quadratic expression has to be convex, namely Q is a
   *    positive semidefinite matrix, and the quadratic expression needs
   *    to be non-negative for any x.
   * @throws std::exception if the preconditions are not satisfied.
   *
   * For example, to add the rotated Lorentz cone constraint
   *
   *     (x+1)(x+y) >= x²+z²+2z+5
   *     x+1 >= 0
   *     x+y >= 0
   * The user could call
   * @code{cc}
   * prog.AddRotatedLorentzConeConstraint(x+1, x+y, pow(x, 2) + pow(z, 2) +
   * 2*z+5);
   * @endcode
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const symbolic::Expression& linear_expression1,
      const symbolic::Expression& linear_expression2,
      const symbolic::Expression& quadratic_expression, double tol = 0);

  /**
   * Adds a constraint that a symbolic expression `v` is in the rotated
   * Lorentz cone, i.e.,
   * \f[
   * v_0v_1 \ge v_2^2 + ... + v_{n-1}^2\\
   * v_0 \ge 0, v_1 \ge 0
   * \f]
   * @param v A linear expression of variables, \f$ v = A x + b\f$, where \f$ A,
   * b \f$ are given matrices of the correct size, \f$ x \f$ is the vector of
   * decision variables.
   * @retval binding The newly added rotated Lorentz cone constraint, together
   * with the bound variables.
   *
   * For example, to add the rotated Lorentz cone constraint
   *
   *     (x+1)(x+y) >= x²+z²+2z+5 = x² + (z+1)² + 2²
   *     x+1 >= 0
   *     x+y >= 0
   * The user could call
   * @code{cc}
   * Eigen::Matrix<symbolic::Expression, 5, 1> v;
   * v << x+1, x+y, x, z+1, 2;
   * prog.AddRotatedLorentzConeConstraint(v);
   * @endcode
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& v);

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)² + ... + z(n-1)²
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size of the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b, const VariableRefList& vars) {
    return AddRotatedLorentzConeConstraint(A, b,
                                           ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a rotated Lorentz cone constraint referencing potentially a subset
   * of decision variables, The linear expression @f$ z=Ax+b @f$ is in rotated
   * Lorentz cone.
   * A vector \f$ z \in\mathbb{R}^n \f$ is in the rotated Lorentz cone, if
   * <!--
   * z(0)*z(1) >= z(2)² + ... + z(n-1)²
   * -->
   * @f[
   * z_0z_1 \ge z_2^2 + ... + z_{n-1}^2
   * @f]
   * where @f$ A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n@f$ are given
   * matrices.
   * @param A A matrix whose number of columns equals to the size of the
   * decision variables.
   * @param b A vector whose number of rows equals to the size of the decision
   * variables.
   * @param vars The decision variables on which the constraint is imposed.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Impose that a vector @f$ x\in\mathbb{R}^m @f$ is in rotated Lorentz cone.
   * Namely
   * @f[
   * x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\\
   * x_0 \ge 0, x_1 \ge 0
   * @f]
   * <!-->
   * x(0)*x(1) >= x(2)^2 + ... x(m-1)^2
   * x(0) >= 0, x(1) >= 0
   * <-->
   * @param vars The stacked column of vars lies in the rotated Lorentz cone.
   * @return The newly added rotated Lorentz cone constraint.
   */
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const VariableRefList& vars) {
    return AddRotatedLorentzConeConstraint(ConcatenateVariableRefList(vars));
  }

  /**
   * Impose that a vector @f$ x\in\mathbb{R}^m @f$ is in rotated Lorentz cone.
   * Namely
   * @f[
   * x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\\
   * x_0 \ge 0, x_1 \ge 0
   * @f]
   * <!-->
   * x(0)*x(1) >= x(2)^2 + ... x(m-1)^2
   * x(0) >= 0, x(1) >= 0
   * <-->
   * @param vars The stacked column of vars lies in the rotated Lorentz cone.
   * @return The newly added rotated Lorentz cone constraint.
   */
  template <int rows>
  Binding<RotatedLorentzConeConstraint> AddRotatedLorentzConeConstraint(
      const Eigen::MatrixBase<VectorDecisionVariable<rows>>& vars) {
    Eigen::Matrix<double, rows, rows> A(vars.rows(), vars.rows());
    A.setIdentity();
    Eigen::Matrix<double, rows, 1> b(vars.rows());
    b.setZero();
    return AddRotatedLorentzConeConstraint(A, b, vars);
  }

  /** Add the convex quadratic constraint 0.5xᵀQx + bᵀx + c <= 0 as a
   * rotated Lorentz cone constraint [rᵀx+s, 1, Px+q] is in the rotated Lorentz
   * cone. When solving the optimization problem using conic solvers (like
   * Mosek, Gurobi, SCS, etc), it is numerically preferable to impose the
   * convex quadratic constraint as rotated Lorentz cone constraint. See
   * https://docs.mosek.com/11.1/capi/prob-def-quadratic.html#a-recommendation
   * @throw exception if this quadratic constraint is not convex (Q is not
   * positive semidefinite)
   * @param Q The Hessian of the quadratic constraint. Should be positive
   * semidefinite.
   * @param b The linear coefficient of the quadratic constraint.
   * @param c The constant term of the quadratic constraint.
   * @param vars x in the documentation above.
   * @param psd_tol If the minimal eigenvalue of Q is smaller than -psd_tol,
   * then throw an exception. @default = 0.
   */
  Binding<RotatedLorentzConeConstraint>
  AddQuadraticAsRotatedLorentzConeConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& Q,
      const Eigen::Ref<const Eigen::VectorXd>& b, double c,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars,
      double psd_tol = 0.);

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearComplementarityConstraint> AddConstraint(
      const Binding<LinearComplementarityConstraint>& binding);

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddLinearComplementarityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& M,
      const Eigen::Ref<const Eigen::VectorXd>& q, const VariableRefList& vars) {
    return AddLinearComplementarityConstraint(M, q,
                                              ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a linear complementarity constraints referencing a subset of
   * the decision variables.
   */
  Binding<LinearComplementarityConstraint> AddLinearComplementarityConstraint(
      const Eigen::Ref<const Eigen::MatrixXd>& M,
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<Constraint> AddPolynomialConstraint(
      const Eigen::Ref<const MatrixX<Polynomiald>>& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::Ref<const Eigen::MatrixXd>& lb,
      const Eigen::Ref<const Eigen::MatrixXd>& ub,
      const VariableRefList& vars) {
    return AddPolynomialConstraint(polynomials, poly_vars, lb, ub,
                                   ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a polynomial constraint to the program referencing a subset
   * of the decision variables (defined in the vars parameter).
   */
  Binding<Constraint> AddPolynomialConstraint(
      const Eigen::Ref<const MatrixX<Polynomiald>>& polynomials,
      const std::vector<Polynomiald::VarType>& poly_vars,
      const Eigen::Ref<const Eigen::MatrixXd>& lb,
      const Eigen::Ref<const Eigen::MatrixXd>& ub,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      const Binding<PositiveSemidefiniteConstraint>& binding);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<PositiveSemidefiniteConstraint> AddConstraint(
      std::shared_ptr<PositiveSemidefiniteConstraint> con,
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix.
   *
   * @throws std::exception in Debug mode if @p symmetric_matrix_var is not
   * symmetric.
   * @param symmetric_matrix_var A symmetric MatrixDecisionVariable object.
   */
  Binding<PositiveSemidefiniteConstraint> AddPositiveSemidefiniteConstraint(
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var);

  /**
   * Adds a positive semidefinite constraint on a symmetric matrix of symbolic
   * expressions @p e. We create a new symmetric matrix of variables M being
   * positive semidefinite, with the linear equality constraint e == M.
   * @param e Imposes constraint "e is positive semidefinite".
   * @pre e is symmetric.
   * @pre e(i, j) is linear for all i, j
   *
   * @return The newly added positive semidefinite constraint, with the bound
   * variable M that are also newly added.
   *
   * For example, to add a constraint that
   *
   *     ⌈x + 1  2x + 3 x+y⌉
   *     |2x+ 3       2   0| is positive semidefinite
   *     ⌊x + y       0   x⌋
   * The user could call
   * @code{cc}
   * Matrix3<symbolic::Expression> e
   * e << x+1, 2*x+3, x+y,
   *      2*x+3,   2,   0,
   *      x+y,     0,   x;
   * prog.AddPositiveSemidefiniteConstraint(e);
   * @endcode
   * @note This function will add additional variables and linear equality
   * constraints. Consider calling AddLinearMatrixInequalityConstraint(e), which
   * doesn't introduce new variables or linear equality constraints.
   */
  Binding<PositiveSemidefiniteConstraint> AddPositiveSemidefiniteConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& e);

  /**
   * Adds a constraint that the principal submatrix of a symmetric matrix
   * composed of the indices in minor_indices is positive semidefinite.
   *
   * @pre The passed @p symmetric_matrix_var is a symmetric matrix.
   * @pre All values in  `minor_indices` lie in the range [0,
   * symmetric_matrix_var.rows() - 1].
   * @param symmetric_matrix_var A symmetric MatrixDecisionVariable object.
   * @see AddPositiveSemidefiniteConstraint
   */
  Binding<PositiveSemidefiniteConstraint> AddPrincipalSubmatrixIsPsdConstraint(
      const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var,
      const std::set<int>& minor_indices);

  /**
   * Adds a constraint the that the principal submatrix of a symmetric matrix of
   * expressions composed of the indices in minor_indices is positive
   * semidefinite.
   *
   * @pre The passed @p symmetric_matrix_var is a symmetric matrix.
   * @pre All values in  `minor_indices` lie in the range [0,
   * symmetric_matrix_var.rows() - 1].
   * @param e Imposes constraint "e is positive semidefinite".
   * @see AddLinearMatrixInequalityConstraint.
   * @note the return type is Binding<LinearMatrixInequalityConstraint>,
   * different from the overloaded function above which returns
   * Binding<PositiveSemidefiniteConstraint>. We impose the constraint as an LMI
   * so as to add fewer additional variables and constraints.
   */
  Binding<LinearMatrixInequalityConstraint>
  AddPrincipalSubmatrixIsPsdConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& e,
      const std::set<int>& minor_indices);

  /**
   * Adds a linear matrix inequality constraint to the program.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<LinearMatrixInequalityConstraint> AddConstraint(
      const Binding<LinearMatrixInequalityConstraint>& binding);

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      std::vector<Eigen::MatrixXd> F, const VariableRefList& vars) {
    return AddLinearMatrixInequalityConstraint(
        std::move(F), ConcatenateVariableRefList(vars));
  }

  /**
   * Adds a linear matrix inequality constraint to the program.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      std::vector<Eigen::MatrixXd> F,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Adds a linear matrix inequality constraint on a symmetric matrix of
   * symbolic expressions `X`, namely `X` is positive semidefinite, and each
   * entry in `X` is a linear (affine) expression of decision variables.
   *
   * @param X Imposes constraint "X is positive semidefinite".
   * @pre X is symmetric.
   * @pre X(i, j) is linear (affine) for all i, j
   *
   * @return The newly added linear matrix inequality constraint.
   */
  Binding<LinearMatrixInequalityConstraint> AddLinearMatrixInequalityConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * Adds the constraint that a symmetric matrix is diagonally dominant with
   * non-negative diagonal entries.
   * A symmetric matrix X is diagonally dominant with non-negative diagonal
   * entries if
   * X(i, i) >= ∑ⱼ |X(i, j)| ∀ j ≠ i
   * namely in each row, the diagonal entry is larger than the sum of the
   * absolute values of all other entries in the same row. A matrix being
   * diagonally dominant with non-negative diagonals is a sufficient (but not
   * necessary) condition of a matrix being positive semidefinite.
   * Internally we will create a matrix Y as slack variables, such that Y(i, j)
   * represents the absolute value |X(i, j)| ∀ j ≠ i. The diagonal entries
   * Y(i, i) = X(i, i)
   * The users can refer to "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586
   * @param X The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
   * X.
   * @return Y The slack variable. Y(i, j) represents |X(i, j)| ∀ j ≠ i, with
   * the constraint Y(i, j) >= X(i, j) and Y(i, j) >= -X(i, j). Y is a symmetric
   * matrix. The diagonal entries Y(i, i) = X(i, i)
   */
  MatrixX<symbolic::Expression> AddPositiveDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * 1. Tightens the positive semidefinite @p constraint with a positive
   * diagonally dominant constraint.
   * 2. Adds the positive diagonally dominant constraint into this
   * MathematicalProgram.
   * 3. Removes the positive semidefinite @p constraint, if it had already been
   * registered in this MathematicalProgram.
   *
   * This provides a polyhedral (i.e. linear) sufficient, but not
   * necessary, condition for the variables in @p constraint to be positive
   * semidefinite.
   *
   * @pre The decision variables contained in constraint have been registered
   * with this MathematicalProgram.
   * @return The return of AddPositiveDiagonallyDominantMatrixConstraint applied
   * to the variables in @p constraint.
   */
  MatrixX<symbolic::Expression> TightenPsdConstraintToDd(
      const Binding<PositiveSemidefiniteConstraint>& constraint);

  /**
   * @anchor add_dd_dual
   * @name Diagonally dominant dual cone constraint
   * Adds the constraint that a symmetric matrix is in the dual cone of the
   * diagonally dominant matrices which is denoted DD*. This set is a polyhedral
   * (linear) outer approximation to the PSD cone. This follows from the fact
   * that since DD ⊆ PSD, then PSD* ⊆ DD*, and since PSD is self-dual, we have
   * that PSD = PSD* and so DD ⊆ PSD = PSD* ⊆ DD*.
   *
   * A symmetric matrix X is in DD* if and only if vᵢᵀXvᵢ ≥ 0 for all vᵢ,
   * where vᵢ is a non-zero vector with at most two entries set to ±1 and all
   * other entries set to 0. There are 4 * (n choose 2) + 2 * n of these
   * vectors, but notice that vᵢᵀXvᵢ = (-vᵢ)ᵀX(vᵢ) and so we only need to add
   * all choices with different partities of which there are 2 * (n choose 2) +
   * n = n². Therefore, if X is a matrix of size n x n, this function adds
   * exactly n² linear constraints.
   *
   * This is a consequence of the characterization of DD given in "Cones
   * of diagonally dominant matrices" by Barker and Carlson which can be found
   * at https://msp.org/pjm/1975/57-1/p03.xhtml.
   */
  //@{
  /**
   * This is an overloaded variant of @ref add_dd_dual
   * "diagonally dominant dual cone constraint"
   * @param X The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
   * X.
   * @pre X(i, j) should be a linear expression of decision variables.
   * @return A linear constraint of size n² encoding vᵢᵀXvᵢ ≥ 0
   *
   * @pydrake_mkdoc_identifier{expression}
   */
  Binding<LinearConstraint>
  AddPositiveDiagonallyDominantDualConeMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * This is an overloaded variant of @ref add_dd_dual
   * "diagonally dominant dual cone constraint"
   * @param X The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
   * X.
   * @return A linear constraint of size n² encoding vᵢᵀXvᵢ ≥ 0
   *
   * @pydrake_mkdoc_identifier{variable}
   */
  Binding<LinearConstraint>
  AddPositiveDiagonallyDominantDualConeMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Variable>>& X);
  //@}

  /**
   * 1. Relaxes the positive semidefinite @p constraint with a diagonally
   * dominant dual cone constraint.
   * 2. Adds the diagonally dominant dual cone constraint into this
   * MathematicalProgram.
   * 3. Removes the positive semidefinite @p constraint, if it had already been
   * registered in this MathematicalProgram.
   *
   * This provides a polyhedral (i.e. linear) necessary, but not
   * sufficient, condition for the variables in @p constraint to be positive
   * semidefinite.
   *
   * @pre The decision variables contained in constraint have been registered
   * with this MathematicalProgram.
   * @return The return of AddPositiveDiagonallyDominantDualConeMatrixConstraint
   * applied to the variables in @p constraint.
   */
  Binding<LinearConstraint> RelaxPsdConstraintToDdDualCone(
      const Binding<PositiveSemidefiniteConstraint>& constraint);

  /**
   * @anchor addsdd
   * @name     Scaled diagonally dominant matrix constraint
   * Adds the constraint that a symmetric matrix is scaled diagonally dominant
   * (sdd). A matrix X is sdd if there exists a diagonal matrix D, such that
   * the product DXD is diagonally dominant with non-negative diagonal entries,
   * namely
   * d(i)X(i, i) ≥ ∑ⱼ |d(j)X(i, j)| ∀ j ≠ i
   * where d(i) = D(i, i).
   * X being sdd is equivalent to the existence of symmetric matrices Mⁱʲ∈ ℝⁿˣⁿ,
   * i < j, such that all entries in Mⁱʲ are 0, except Mⁱʲ(i, i), Mⁱʲ(i, j),
   * Mⁱʲ(j, j). (Mⁱʲ(i, i), Mⁱʲ(j, j), Mⁱʲ(i, j)) is in the rotated
   * Lorentz cone, and X = ∑ᵢⱼ Mⁱʲ.
   *
   * The users can refer to "DSOS and SDSOS Optimization: More Tractable
   * Alternatives to Sum of Squares and Semidefinite Optimization" by Amir Ali
   * Ahmadi and Anirudha Majumdar, with arXiv link
   * https://arxiv.org/abs/1706.02586.
   */
  //@{
  /**
   * This is an overloaded variant of @ref addsdd
   * "scaled diagonally dominant matrix constraint"
   * @param X The matrix X to be constrained scaled diagonally dominant.
   * X.
   * @pre X(i, j) should be a linear expression of decision variables.
   * @return M A vector of vectors of 2 x 2 symmetric matrices M. For i < j,
   * M[i][j] is
   * <pre>
   * [Mⁱʲ(i, i), Mⁱʲ(i, j)]
   * [Mⁱʲ(i, j), Mⁱʲ(j, j)].
   * </pre>
   * Note that M[i][j](0, 1) = Mⁱʲ(i, j) = (X(i, j) + X(j, i)) / 2
   * for i >= j, M[i][j] is the zero matrix.
   *
   * @pydrake_mkdoc_identifier{expression}
   */
  std::vector<std::vector<Matrix2<symbolic::Expression>>>
  AddScaledDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * This is an overloaded variant of @ref addsdd
   * "scaled diagonally dominant matrix constraint"
   * @param X The symmetric matrix X to be constrained scaled diagonally
   * dominant.
   * @return M For i < j M[i][j] contains the slack variables, mentioned in
   * @ref addsdd "scaled diagonally dominant matrix constraint". For i >= j,
   * M[i][j] contains default-constructed variables (with get_id() == 0).
   *
   * @pydrake_mkdoc_identifier{variable}
   */
  std::vector<std::vector<Matrix2<symbolic::Variable>>>
  AddScaledDiagonallyDominantMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Variable>>& X);
  //@}

  /**
   * 1. Tightens the positive semidefinite @p constraint with a scaled
   * diagonally dominant constraint.
   * 2. Adds the scaled diagonally dominant constraint into this
   * MathematicalProgram.
   * 3. Removes the positive semidefinite @p constraint, if it had already been
   * registered in this MathematicalProgram.
   *
   * This provides a second-order cone sufficient, but not
   * necessary, condition for the variables in @p constraint to be positive
   * semidefinite.
   *
   * @pre The decision variables contained in constraint have been registered
   * with this MathematicalProgram.
   * @return The return of AddScaledDiagonallyDominantMatrixConstraint applied
   * to the variables in @p constraint.
   */
  std::vector<std::vector<Matrix2<symbolic::Variable>>>
  TightenPsdConstraintToSdd(
      const Binding<PositiveSemidefiniteConstraint>& constraint);

  /**
   * @anchor add_sdd_dual
   * @name Scaled diagonally dominant dual cone constraint
   * Adds the constraint that a symmetric matrix is in the dual cone of the
   * scaled diagonally dominant matrices which is denoted SDD*. The set SDD* is
   * an SOCP outer approximation to the PSD cone that is tighter than DD*. This
   * follows from the fact that DD ⊆ SDD ⊆ PSD = PSD* ⊆ SDD* ⊆ DD*.
   *
   * A symmetric matrix X is in SDD* if and only if all 2 x 2 principal minors
   * of X are psd. This can be encoded by ensuring that VᵢⱼᵀXVᵢⱼ is psd for all
   * Vᵢⱼ, where Vᵢⱼ is the n x 2 matrix such that Vᵢⱼ(i, 0) = 1, V(j, 1) = 1,
   * namely Vᵢⱼ = [eᵢ eⱼ].
   * This can be encoded using 1/2 * n * (n-1) RotatedLorentzCone constraints
   * which we return in this function.
   *
   * This can be seen by noting that
   * VᵢⱼᵀXVᵢⱼ = ⌈ Xᵢᵢ Xᵢⱼ⌉
                ⌊ Xⱼᵢ Xⱼⱼ⌋
   * is psd if and only if VⱼᵢᵀXVⱼᵢ as they are simply permutations of each
   * other. Therefore, it suffices to only add the constraint for i ≥ j.
   * Moreover, notice that VᵢᵢᵀXVᵢᵢ = ⌈ Xᵢᵢ 0⌉ ⌊ 0   0⌋ is psd if and only if
   * Xᵢᵢ ≥ 0. This linear constraint is already implied by VᵢⱼᵀXVᵢⱼ is psd for
   * every i ≠ j and so is redundant. Therefore, we only add
   * RotatedLorentzConeConstraints for i > j.
   *
   * This characterization can be found in Section 3.3 of "Sum of Squares Basis
   * Pursuit with Linear and Second Order Cone Programming" by Ahmadi and Hall
   * with arXiv link https://arxiv.org/abs/1510.01597
   */
  //@{
  /**
   * This is an overloaded variant of @ref add_sdd_dual
   * "scaled diagonally dominant dual cone constraint"
   * @param X The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
   * X.
   * @pre X(i, j) should be a linear expression of decision variables.
   * @return A vector of RotatedLorentzConeConstraint constraints of length
   *  1/2 * n * (n-1) encoding VᵢⱼᵀXVᵢⱼ is psd
   *  @pydrake_mkdoc_identifier{expression}
   */
  std::vector<Binding<RotatedLorentzConeConstraint>>
  AddScaledDiagonallyDominantDualConeMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>& X);

  /**
   * This is an overloaded variant of @ref add_sdd_dual
   * "scaled diagonally dominant dual cone constraint"
   * @param X The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
   * X.
   * @return A vector of RotatedLorentzConeConstraint constraints of length
   * 1/2 * n * (n-1) encoding VᵢⱼᵀXVᵢⱼ is psd
   * @pydrake_mkdoc_identifier{variable}
   */
  std::vector<Binding<RotatedLorentzConeConstraint>>
  AddScaledDiagonallyDominantDualConeMatrixConstraint(
      const Eigen::Ref<const MatrixX<symbolic::Variable>>& X);
  //@}

  /**
   * 1. Relaxes the positive semidefinite @p constraint with a scaled diagonally
   * dominant dual cone constraint.
   * 2. Adds the scaled diagonally dominant dual cone constraint into this
   * MathematicalProgram.
   * 3. Removes the positive semidefinite @p constraint, if it had already been
   * registered in this MathematicalProgram.
   *
   * This provides a second-order cone necessary, but not
   * sufficient, condition for the variables in @p constraint to be positive
   * semidefinite.
   *
   * @pre The decision variables contained in constraint have been registered
   * with this MathematicalProgram.
   * @return The return of AddScaledDiagonallyDominantDualConeMatrixConstraint
   * applied to the variables in @p constraint.
   */
  std::vector<Binding<RotatedLorentzConeConstraint>>
  RelaxPsdConstraintToSddDualCone(
      const Binding<PositiveSemidefiniteConstraint>& constraint);

  /**
   * Adds constraints that a given polynomial @p p is a sums-of-squares (SOS),
   * that is, @p p can be decomposed into `mᵀQm`, where m is the @p
   * monomial_basis. It returns the coefficients matrix Q, which is positive
   * semidefinite.
   * @param type The type of the polynomial. @default is kSos, but the user can
   * also use kSdsos and kDsos. Refer to NonnegativePolynomial for details on
   * different types of sos polynomials.
   * @param gram_name The name of the gram matrix for print out.
   *
   * @note It calls `Reparse` to enforce `p` to have this MathematicalProgram's
   * indeterminates if necessary.
   */
  MatrixXDecisionVariable AddSosConstraint(
      const symbolic::Polynomial& p,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      NonnegativePolynomial type = NonnegativePolynomial::kSos,
      const std::string& gram_name = "S");

  /**
   * Adds constraints that a given polynomial @p p is a sums-of-squares (SOS),
   * that is, @p p can be decomposed into `mᵀQm`, where m is a monomial
   * basis selected from the sparsity of @p p. It returns a pair of constraint
   * bindings expressing:
   *  - The coefficients matrix Q, which is positive semidefinite.
   *  - The monomial basis m.
   * @param type The type of the polynomial. @default is kSos, but the user can
   * also use kSdsos and kDsos. Refer to NonnegativePolynomial for the details
   * on different type of sos polynomials.
   * @param gram_name The name of the gram matrix for print out.
   *
   * @note It calls `Reparse` to enforce `p` to have this MathematicalProgram's
   * indeterminates if necessary.
   */
  std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>>
  AddSosConstraint(const symbolic::Polynomial& p,
                   NonnegativePolynomial type = NonnegativePolynomial::kSos,
                   const std::string& gram_name = "S");

  /**
   * Adds constraints that a given symbolic expression @p e is a
   * sums-of-squares (SOS), that is, @p p can be decomposed into `mᵀQm`,
   * where m is the @p monomial_basis.  Note that it decomposes @p e into a
   * polynomial with respect to `indeterminates()` in this mathematical
   * program. It returns the coefficients matrix Q, which is positive
   * semidefinite.
   * @param type Refer to NonnegativePolynomial class documentation.
   * @param gram_name The name of the gram matrix for print out.
   */
  MatrixXDecisionVariable AddSosConstraint(
      const symbolic::Expression& e,
      const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis,
      NonnegativePolynomial type = NonnegativePolynomial::kSos,
      const std::string& gram_name = "S");

  /**
   * Adds constraints that a given symbolic expression @p e is a sums-of-squares
   * (SOS), that is, @p e can be decomposed into `mᵀQm`. Note that it decomposes
   * @p e into a polynomial with respect to `indeterminates()` in this
   * mathematical program. It returns a pair expressing:
   *  - The coefficients matrix Q, which is positive semidefinite.
   *  - The monomial basis m.
   * @param type Refer to NonnegativePolynomial class documentation.
   * @param gram_name The name of the gram matrix for print out.
   */
  std::pair<MatrixXDecisionVariable, VectorX<symbolic::Monomial>>
  AddSosConstraint(const symbolic::Expression& e,
                   NonnegativePolynomial type = NonnegativePolynomial::kSos,
                   const std::string& gram_name = "S");

  /**
   * Constraining that two polynomials are the same (i.e., they have the same
   * coefficients for each monomial). This function is often used in
   * sum-of-squares optimization.
   * We will impose the linear equality constraint that the coefficient of a
   * monomial in @p p1 is the same as the coefficient of the same monomial in @p
   * p2.
   * @param p1 Note that p1's indeterminates should have been registered as
   * indeterminates in this MathematicalProgram object, and p1's coefficients
   * are affine functions of decision variables in this MathematicalProgram
   * object.
   * @param p2 Note that p2's indeterminates should have been registered as
   * indeterminates in this MathematicalProgram object, and p2's coefficients
   * are affine functions of decision variables in this MathematicalProgram
   * object.
   *
   * @note It calls `Reparse` to enforce `p1` and `p2` to have this
   * MathematicalProgram's indeterminates.
   */
  std::vector<Binding<LinearEqualityConstraint>>
  AddEqualityConstraintBetweenPolynomials(const symbolic::Polynomial& p1,
                                          const symbolic::Polynomial& p2);

  /**
   * Adds the exponential cone constraint that
   * z = binding.evaluator()->A() * binding.variables() +
   *     binding.evaluator()->b()
   * should be in the exponential cone. Namely
   * {(z₀, z₁, z₂) | z₀ ≥ z₁ * exp(z₂ / z₁), z₁ > 0}.
   * @param binding The binding of ExponentialConeConstraint and its bound
   * variables.
   *
   * @exclude_from_pydrake_mkdoc{Not bound in pydrake.}
   */
  Binding<ExponentialConeConstraint> AddConstraint(
      const Binding<ExponentialConeConstraint>& binding);

  /**
   * Adds an exponential cone constraint, that z = A * vars + b should be in
   * the exponential cone. Namely {z₀, z₁, z₂ | z₀ ≥ z₁ * exp(z₂ / z₁), z₁ >
   * 0}, or equivalently (using the logarithm function), {z₀, z₁, z₂ | z₂ ≤ z₁ *
   * log(z₀ / z₁), z₀ > 0, z₁ > 0}.
   * @param A The A matrix in the documentation above. A must have 3 rows.
   * @param b The b vector in the documentation above.
   * @param vars The variables bound with this constraint.
   */
  Binding<ExponentialConeConstraint> AddExponentialConeConstraint(
      const Eigen::Ref<const Eigen::SparseMatrix<double>>& A,
      const Eigen::Ref<const Eigen::Vector3d>& b,
      const Eigen::Ref<const VectorXDecisionVariable>& vars);

  /**
   * Add the constraint that z is in the exponential cone.
   * @param z The expression in the exponential cone.
   * @pre each entry in `z` is a linear expression of the decision variables.
   */
  Binding<ExponentialConeConstraint> AddExponentialConeConstraint(
      const Eigen::Ref<const Vector3<symbolic::Expression>>& z);

  /**
   * Gets the initial guess for a single variable.
   * @pre @p decision_variable has been registered in the optimization program.
   * @throws std::exception if the pre condition is not satisfied.
   */
  [[nodiscard]] double GetInitialGuess(
      const symbolic::Variable& decision_variable) const;

  /**
   * Gets the initial guess for some variables.
   * @pre Each variable in @p decision_variable_mat has been registered in the
   * optimization program.
   * @throws std::exception if the pre condition is not satisfied.
   */
  template <typename Derived>
    requires(std::is_same_v<typename Derived::Scalar, symbolic::Variable>)
  MatrixLikewise<double, Derived> GetInitialGuess(
      const Eigen::MatrixBase<Derived>& decision_variable_mat) const {
    MatrixLikewise<double, Derived> decision_variable_values(
        decision_variable_mat.rows(), decision_variable_mat.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        decision_variable_values(i, j) =
            GetInitialGuess(decision_variable_mat(i, j));
      }
    }
    return decision_variable_values;
  }

  /**
   * Sets the initial guess for a single variable @p decision_variable.
   * The guess is stored as part of this program.
   * @pre decision_variable is a registered decision variable in the program.
   * @throws std::exception if precondition is not satisfied.
   */
  void SetInitialGuess(const symbolic::Variable& decision_variable,
                       double variable_guess_value);

  /**
   * Sets the initial guess for the decision variables stored in
   * @p decision_variable_mat to be @p x0.
   * The guess is stored as part of this program.
   */
  template <typename DerivedA, typename DerivedB>
  void SetInitialGuess(const Eigen::MatrixBase<DerivedA>& decision_variable_mat,
                       const Eigen::MatrixBase<DerivedB>& x0) {
    DRAKE_DEMAND(decision_variable_mat.rows() == x0.rows());
    DRAKE_DEMAND(decision_variable_mat.cols() == x0.cols());
    for (int i = 0; i < decision_variable_mat.rows(); ++i) {
      for (int j = 0; j < decision_variable_mat.cols(); ++j) {
        SetInitialGuess(decision_variable_mat(i, j), x0(i, j));
      }
    }
  }

  /**
   * Set the initial guess for ALL decision variables.
   * Note that variables begin with a default initial guess of NaN to indicate
   * that no guess is available.
   * @param x0 A vector of appropriate size (num_vars() x 1).
   */
  template <typename Derived>
  void SetInitialGuessForAllVariables(const Eigen::MatrixBase<Derived>& x0) {
    DRAKE_ASSERT(x0.rows() == num_vars() && x0.cols() == 1);
    x_initial_guess_ = x0;
  }

  /**
   * Updates the value of a single @p decision_variable inside the @p values
   * vector to be @p decision_variable_new_value.
   * The other decision variables' values in @p values are unchanged.
   * @param decision_variable a registered decision variable in this program.
   * @param decision_variable_new_value the variable's new values.
   * @param[in,out] values The vector to be tweaked; must be of size num_vars().
   */
  void SetDecisionVariableValueInVector(
      const symbolic::Variable& decision_variable,
      double decision_variable_new_value,
      EigenPtr<Eigen::VectorXd> values) const;

  /**
   * Updates the values of some @p decision_variables inside the @p values
   * vector to be @p decision_variables_new_values.
   * The other decision variables' values in @p values are unchanged.
   * @param decision_variables registered decision variables in this program.
   * @param decision_variables_new_values the variables' respective new values;
   *   must have the same rows() and cols() sizes and @p decision_variables.
   * @param[in,out] values The vector to be tweaked; must be of size num_vars().
   */
  void SetDecisionVariableValueInVector(
      const Eigen::Ref<const MatrixXDecisionVariable>& decision_variables,
      const Eigen::Ref<const Eigen::MatrixXd>& decision_variables_new_values,
      EigenPtr<Eigen::VectorXd> values) const;

  /**
   * @anchor set_solver_option
   * @name Set solver options
   * Set the options (parameters) for a specific solver. Refer to SolverOptions
   * class for more details on the supported options of each solver.
   */

  //@{
  /**
   * See @ref set_solver_option for more details.
   * Set the double-valued options.
   * @pydrake_mkdoc_identifier{double_option}
   */
  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, double option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /**
   * See @ref set_solver_option for more details.
   * Set the integer-valued options.
   * @pydrake_mkdoc_identifier{int_option}
   */
  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option, int option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /**
   * See @ref set_solver_option for more details.
   * Set the string-valued options.
   * @pydrake_mkdoc_identifier{string_option}
   */
  void SetSolverOption(const SolverId& solver_id,
                       const std::string& solver_option,
                       const std::string& option_value) {
    solver_options_.SetOption(solver_id, solver_option, option_value);
  }

  /**
   * Overwrite the stored solver options inside MathematicalProgram with the
   * provided solver options.
   */
  void SetSolverOptions(const SolverOptions& solver_options) {
    solver_options_ = solver_options;
  }
  //@}

  /**
   * Returns the solver options stored inside MathematicalProgram.
   */
  const SolverOptions& solver_options() const { return solver_options_; }

  /**
   * Getter for all callbacks.
   */
  const std::vector<Binding<VisualizationCallback>>& visualization_callbacks()
      const {
    return visualization_callbacks_;
  }

  /**
   * Getter for all generic costs.
   */
  const std::vector<Binding<Cost>>& generic_costs() const {
    return generic_costs_;
  }  // e.g. for snopt_user_fun

  /**
   * Getter for all generic constraints
   */
  const std::vector<Binding<Constraint>>& generic_constraints() const {
    return generic_constraints_;
  }  // e.g. for snopt_user_fun

  /**
   * Getter for linear equality constraints. Note that this only includes
   * constraints that were added explicitly as LinearEqualityConstraint or
   * which were added symbolically (and their equality constraint nature was
   * uncovered). There may be bounding_box_constraints() and
   * linear_constraints() whose lower bounds also equal their upper bounds.
   */
  const std::vector<Binding<LinearEqualityConstraint>>&
  linear_equality_constraints() const {
    return linear_equality_constraints_;
  }

  /** Getter for linear costs. */
  const std::vector<Binding<LinearCost>>& linear_costs() const {
    return linear_costs_;
  }

  /** Getter for quadratic costs. */
  const std::vector<Binding<QuadraticCost>>& quadratic_costs() const {
    return quadratic_costs_;
  }

  /** Getter for l2norm costs. */
  const std::vector<Binding<L2NormCost>>& l2norm_costs() const {
    return l2norm_costs_;
  }

  /** Getter for linear *inequality* constraints. Note that this does not
   * include linear_equality_constraints() nor bounding_box_constraints(). See
   * also GetAllLinearConstraints(). */
  const std::vector<Binding<LinearConstraint>>& linear_constraints() const {
    return linear_constraints_;
  }

  /** Getter for quadratic constraints. */
  const std::vector<Binding<QuadraticConstraint>>& quadratic_constraints()
      const {
    return quadratic_constraints_;
  }

  /** Getter for Lorentz cone constraints. */
  const std::vector<Binding<LorentzConeConstraint>>& lorentz_cone_constraints()
      const {
    return lorentz_cone_constraint_;
  }

  /** Getter for rotated Lorentz cone constraints. */
  const std::vector<Binding<RotatedLorentzConeConstraint>>&
  rotated_lorentz_cone_constraints() const {
    return rotated_lorentz_cone_constraint_;
  }

  /** Getter for positive semidefinite constraints. */
  const std::vector<Binding<PositiveSemidefiniteConstraint>>&
  positive_semidefinite_constraints() const {
    return positive_semidefinite_constraint_;
  }

  /** Getter for linear matrix inequality constraints. */
  const std::vector<Binding<LinearMatrixInequalityConstraint>>&
  linear_matrix_inequality_constraints() const {
    return linear_matrix_inequality_constraint_;
  }

  /** Getter for exponential cone constraints. */
  const std::vector<Binding<ExponentialConeConstraint>>&
  exponential_cone_constraints() const {
    return exponential_cone_constraints_;
  }

  /** Getter for all bounding box constraints */
  const std::vector<Binding<BoundingBoxConstraint>>& bounding_box_constraints()
      const {
    return bbox_constraints_;
  }

  /** Getter for all linear complementarity constraints.*/
  const std::vector<Binding<LinearComplementarityConstraint>>&
  linear_complementarity_constraints() const {
    return linear_complementarity_constraints_;
  }

  /**
   * Getter returning all costs.
   * @returns Vector of all cost bindings.
   * @note The group ordering may change as more cost types are added.
   */
  [[nodiscard]] std::vector<Binding<Cost>> GetAllCosts() const;

  /**
   * Getter returning all linear constraints (both linear equality and
   * inequality constraints). Note that this does *not* include bounding box
   * constraints, which are technically also linear.
   * @returns Vector of all linear constraint bindings.
   */
  [[nodiscard]] std::vector<Binding<LinearConstraint>> GetAllLinearConstraints()
      const;

  /**
   * Getter for returning all constraints.
   * @returns Vector of all constraint bindings.
   * @note The group ordering may change as more constraint types are added.
   */
  [[nodiscard]] std::vector<Binding<Constraint>> GetAllConstraints() const;

  /** Getter for number of variables in the optimization program */
  int num_vars() const { return decision_variables_.size(); }

  /** Gets the number of indeterminates in the optimization program */
  int num_indeterminates() const { return indeterminates_.size(); }

  /** Getter for the initial guess */
  const Eigen::VectorXd& initial_guess() const { return x_initial_guess_; }

  /** Returns the index of the decision variable. Internally the solvers thinks
   * all variables are stored in an array, and it accesses each individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p var is a decision variable in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  [[nodiscard]] int FindDecisionVariableIndex(
      const symbolic::Variable& var) const;

  /**
   * Returns the indices of the decision variables. Internally the solvers
   * thinks all variables are stored in an array, and it accesses each
   * individual
   * variable using its index. This index is used when adding constraints
   * and costs for each solver.
   * @pre{@p vars are decision variables in the mathematical program, otherwise
   * this function throws a runtime error.}
   */
  [[nodiscard]] std::vector<int> FindDecisionVariableIndices(
      const Eigen::Ref<const VectorXDecisionVariable>& vars) const;

  /** Returns the index of the indeterminate. Internally a solver
   * thinks all indeterminates are stored in an array, and it accesses each
   * individual indeterminate using its index. This index is used when adding
   * constraints and costs for each solver.
   * @pre @p var is a indeterminate in the mathematical program,
   * otherwise this function throws a runtime error.
   */
  [[nodiscard]] size_t FindIndeterminateIndex(
      const symbolic::Variable& var) const;

  /**
   * Evaluates the value of some binding, for some input value for all
   * decision variables.
   * @param binding A Binding whose variables are decision variables in this
   * program.
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @throws std::exception if the size of `prog_var_vals` is invalid.
   */
  template <typename C, typename DerivedX>
    requires(is_eigen_vector<DerivedX>::value)
  VectorX<typename DerivedX::Scalar> EvalBinding(
      const Binding<C>& binding,
      const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    using Scalar = typename DerivedX::Scalar;
    if (prog_var_vals.rows() != num_vars()) {
      std::ostringstream oss;
      oss << "The input binding variable is not in the right size. Expects "
          << num_vars() << " rows, but it actually has " << prog_var_vals.rows()
          << " rows.\n";
      throw std::logic_error(oss.str());
    }
    VectorX<Scalar> binding_x(binding.GetNumElements());
    VectorX<Scalar> binding_y(binding.evaluator()->num_outputs());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      binding_x(i) =
          prog_var_vals(FindDecisionVariableIndex(binding.variables()(i)));
    }
    binding.evaluator()->Eval(binding_x, &binding_y);
    return binding_y;
  }

  /**
   * Evaluates a set of bindings (plural version of `EvalBinding`).
   * @param bindings List of bindings.
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @return All binding values, concatenated into a single vector.
   * @throws std::exception if the size of `prog_var_vals` is invalid.
   */
  template <typename C, typename DerivedX>
    requires(is_eigen_vector<DerivedX>::value)
  VectorX<typename DerivedX::Scalar> EvalBindings(
      const std::vector<Binding<C>>& bindings,
      const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    // TODO(eric.cousineau): Minimize memory allocations when it becomes a
    // major performance bottleneck.
    using Scalar = typename DerivedX::Scalar;
    int num_y{};
    for (auto& binding : bindings) {
      num_y += binding.evaluator()->num_outputs();
    }
    VectorX<Scalar> y(num_y);
    int offset_y{};
    for (auto& binding : bindings) {
      VectorX<Scalar> binding_y = EvalBinding(binding, prog_var_vals);
      y.segment(offset_y, binding_y.size()) = binding_y;
      offset_y += binding_y.size();
    }
    DRAKE_DEMAND(offset_y == num_y);
    return y;
  }

  /**
   * Given the value of all decision variables, namely
   * this.decision_variable(i) takes the value prog_var_vals(i), returns the
   * vector that contains the value of the variables in binding.variables().
   * @param binding binding.variables() must be decision variables in this
   * MathematicalProgram.
   * @param prog_var_vals The value of ALL the decision variables in this
   * program.
   * @return binding_variable_vals binding_variable_vals(i) is the value of
   * binding.variables()(i) in prog_var_vals.
   */
  template <typename C, typename DerivedX>
    requires(is_eigen_vector<DerivedX>::value)
  VectorX<typename DerivedX::Scalar> GetBindingVariableValues(
      const Binding<C>& binding,
      const Eigen::MatrixBase<DerivedX>& prog_var_vals) const {
    DRAKE_DEMAND(prog_var_vals.rows() == num_vars());
    VectorX<typename DerivedX::Scalar> result(binding.GetNumElements());
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      result(i) =
          prog_var_vals(FindDecisionVariableIndex(binding.variables()(i)));
    }
    return result;
  }

  /** Evaluates all visualization callbacks registered with the
   * MathematicalProgram.
   *
   * @param prog_var_vals The value of all the decision variables in this
   * program.
   * @throws std::exception if the size does not match.
   */
  void EvalVisualizationCallbacks(
      const Eigen::Ref<const Eigen::VectorXd>& prog_var_vals) const;

  /**
   * Evaluates the evaluator in @p binding at the initial guess.
   * @return The value of @p binding at the initial guess.
   */
  template <typename C>
  Eigen::VectorXd EvalBindingAtInitialGuess(const Binding<C>& binding) const {
    return EvalBinding(binding, x_initial_guess_);
  }

  /**
   * Evaluates CheckSatisfied for the constraint in @p binding using the value
   * of ALL of the decision variables in this program.
   * @throws std::exception if the size of `prog_var_vals` is invalid.
   */
  [[nodiscard]] bool CheckSatisfied(
      const Binding<Constraint>& binding,
      const Eigen::Ref<const Eigen::VectorXd>& prog_var_vals,
      double tol = 1e-6) const;

  /**
   * Evaluates CheckSatisfied for the all of the constraints in @p binding using
   * the value of ALL of the decision variables in this program.
   * @returns true iff all of the constraints are satisfied.
   * @throws std::exception if the size of `prog_var_vals` is invalid.
   * @pydrake_mkdoc_identifier{vector}
   */
  [[nodiscard]] bool CheckSatisfied(
      const std::vector<Binding<Constraint>>& bindings,
      const Eigen::Ref<const Eigen::VectorXd>& prog_var_vals,
      double tol = 1e-6) const;

  /**
   * Evaluates CheckSatisfied for the constraint in @p binding at the initial
   * guess.
   */
  [[nodiscard]] bool CheckSatisfiedAtInitialGuess(
      const Binding<Constraint>& binding, double tol = 1e-6) const;

  /**
   * Evaluates CheckSatisfied for the all of the constraints in @p bindings at
   * the initial guess.
   * @returns true iff all of the constraints are satisfied.
   * @pydrake_mkdoc_identifier{vector}
   */
  [[nodiscard]] bool CheckSatisfiedAtInitialGuess(
      const std::vector<Binding<Constraint>>& bindings,
      double tol = 1e-6) const;

  /** Getter for all decision variables in the program. */
  Eigen::Map<const VectorX<symbolic::Variable>> decision_variables() const {
    return Eigen::Map<const VectorX<symbolic::Variable>>(
        decision_variables_.data(), decision_variables_.size());
  }

  /** Getter for the decision variable with index @p i in the program. */
  const symbolic::Variable& decision_variable(int i) const {
    DRAKE_ASSERT(i >= 0);
    DRAKE_ASSERT(i < static_cast<int>(decision_variables_.size()));
    return decision_variables_[i];
  }

  /** Getter for all indeterminates in the program. */
  Eigen::Map<const VectorX<symbolic::Variable>> indeterminates() const {
    return Eigen::Map<const VectorX<symbolic::Variable>>(
        indeterminates_.data(), indeterminates_.size());
  }

  /** Getter for the indeterminate with index @p i in the program. */
  const symbolic::Variable& indeterminate(int i) const {
    DRAKE_ASSERT(i >= 0);
    DRAKE_ASSERT(i < static_cast<int>(indeterminates_.size()));
    return indeterminates_[i];
  }

  /// Getter for the required capability on the solver, given the
  /// cost/constraint/variable types in the program.
  const ProgramAttributes& required_capabilities() const {
    return required_capabilities_;
  }

  /**
   * Returns the mapping from a decision variable ID to its index in the vector
   * containing all the decision variables in the mathematical program.
   */
  const std::unordered_map<symbolic::Variable::Id, int>&
  decision_variable_index() const {
    return decision_variable_index_;
  }

  /**
   * Returns the mapping from an indeterminate ID to its index in the vector
   * containing all the indeterminates in the mathematical program.
   */
  const std::unordered_map<symbolic::Variable::Id, int>& indeterminates_index()
      const {
    return indeterminates_index_;
  }

  /**
   * @anchor variable_scaling
   * @name Variable scaling
   * Some solvers (e.g. SNOPT) work better if the decision variables values
   * are on the same scale. Hence, internally we scale the variable as
   * snopt_var_value = var_value / scaling_factor.
   * This scaling factor is only used inside the solve, so
   * users don't need to manually scale the variables every time they appears in
   * cost and constraints. When the users set the initial guess, or getting the
   * result from MathematicalProgramResult::GetSolution(), the values are
   * unscaled. Namely, MathematicalProgramResult::GetSolution(var) returns the
   * value of var, not var_value / scaling_factor.
   *
   * The feature of variable scaling is currently only implemented for SNOPT and
   * OSQP.
   */
  //@{
  /**
   * Returns the mapping from a decision variable index to its scaling factor.
   *
   * See @ref variable_scaling "Variable scaling" for more information.
   */
  const std::unordered_map<int, double>& GetVariableScaling() const {
    return var_scaling_map_;
  }

  /**
   * Setter for the scaling @p s of decision variable @p var.
   * @param var the decision variable to be scaled.
   * @param s scaling factor (must be positive).
   *
   * See @ref variable_scaling "Variable scaling" for more information.
   */
  void SetVariableScaling(const symbolic::Variable& var, double s);

  /**
   * Clears the scaling factors for decision variables.
   *
   * See @ref variable_scaling "Variable scaling" for more information.
   */
  void ClearVariableScaling() { var_scaling_map_.clear(); }
  //@}

  /**
   * Remove `var` from this program's decision variable.
   * @note after removing the variable, the indices of some remaining variables
   * inside this MathematicalProgram will change.
   * @return the index of `var` in this optimization program. return -1 if `var`
   * is not a decision variable.
   * @throw exception if `var` is bound with any cost or constraint.
   * @throw exception if `var` is not a decision variable of the program.
   */
  int RemoveDecisionVariable(const symbolic::Variable& var);

  /**
   * @anchor remove_cost_constraint
   * @name    Remove costs, constraints or callbacks.
   * Removes costs, constraints or visualization callbacks from this program. If
   * this program contains multiple costs/constraints/callbacks objects matching
   * the given argument, then all of these costs/constraints/callbacks are
   * removed. If this program doesn't contain the specified
   * cost/constraint/callback, then the code does nothing. We regard two
   * costs/constraints/callbacks being equal, if their evaluators point to the
   * same object, and the associated variables are also the same.
   * @note If two costs/constraints/callbacks represent the same expression, but
   * their evaluators point to different objects, then they are NOT regarded the
   * same. For example, if we have
   * @code{.cc}
   * auto cost1 = prog.AddLinearCost(x[0] + x[1]);
   * auto cost2 = prog.AddLinearCost(x[0] + x[1]);
   * // cost1 and cost2 represent the same cost, but cost1.evaluator() and
   * // cost2.evaluator() point to different objects. So after removing cost1,
   * // cost2 still lives in prog.
   * prog.RemoveCost(cost1);
   * // This will print true.
   * std::cout << (prog.linear_costs()[0] == cost2) << "\n";
   * @endcode
   */

  // @{
  /** Removes @p cost from this mathematical program.
   * See @ref remove_cost_constraint "Remove costs, constraints or callbacks"
   * for more details.
   * @return number of cost objects removed from this program. If this program
   * doesn't contain @p cost, then returns 0. If this program contains multiple
   * @p cost objects, then returns the repetition of @p cost in this program.
   */
  int RemoveCost(const Binding<Cost>& cost);

  /** Removes @p constraint from this mathematical program.
   * See @ref remove_cost_constraint "Remove costs, constraints or callbacks"
   * for more details.
   * @return number of constraint objects removed from this program. If this
   * program doesn't contain @p constraint, then returns 0. If this program
   * contains multiple
   * @p constraint objects, then returns the repetition of @p constraint in this
   * program.
   */
  int RemoveConstraint(const Binding<Constraint>& constraint);

  /** Removes @p callback from this mathematical program.
   * See @ref remove_cost_constraint "Remove costs, constraints or callbacks"
   * for more details.
   * @return number of callback objects removed from this program. If this
   * program doesn't contain @p callback, then returns 0. If this program
   * contains multiple
   * @p callback objects, then returns the repetition of @p callback in this
   * program.
   */
  int RemoveVisualizationCallback(
      const Binding<VisualizationCallback>& callback);
  //@}

 private:
  // Copy constructor is private for use in implementing Clone().
  explicit MathematicalProgram(const MathematicalProgram&);

  static void AppendNanToEnd(int new_var_size, Eigen::VectorXd* vector);

  // Removes a binding of a constraint/constraint, @p removal, from a given
  // vector of bindings, @p existings. If @p removal does not belong to @p
  // existings, then do nothing. If @p removal appears multiple times in @p
  // existings, then all matching terms are removed. After removing @p removal,
  // we check if we need to erase @p affected_capability from
  // required_capabilities_.
  // @return The number of @p removal object in @p existings.
  template <typename C>
  int RemoveCostOrConstraintImpl(const Binding<C>& removal,
                                 ProgramAttribute affected_capability,
                                 std::vector<Binding<C>>* existings);

  /**
   * Check and update if this program requires @p query_capability
   * This method should be called after changing stored
   * costs/constraints/callbacks.
   */
  void UpdateRequiredCapability(ProgramAttribute query_capability);

  template <typename T>
  void NewVariables_impl(
      VarType type, const T& names, bool is_symmetric,
      Eigen::Ref<MatrixXDecisionVariable> decision_variable_matrix) {
    CheckVariableType(type);
    int rows = decision_variable_matrix.rows();
    int cols = decision_variable_matrix.cols();
    DRAKE_ASSERT(!is_symmetric || rows == cols);
    int num_new_vars = 0;
    if (!is_symmetric) {
      num_new_vars = rows * cols;
    } else {
      num_new_vars = rows * (rows + 1) / 2;
    }
    DRAKE_ASSERT(static_cast<int>(names.size()) == num_new_vars);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      decision_variables_.emplace_back(names[i], type);
      const int new_var_index = decision_variables_.size() - 1;
      decision_variable_index_.insert(std::make_pair(
          decision_variables_[new_var_index].get_id(), new_var_index));
      decision_variable_matrix(row_index, col_index) =
          decision_variables_[new_var_index];
      // If the matrix is not symmetric, then store the variable in column
      // major.
      if (!is_symmetric) {
        if (row_index + 1 < rows) {
          ++row_index;
        } else {
          ++col_index;
          row_index = 0;
        }
      } else {
        // If the matrix is symmetric, then the decision variables are the lower
        // triangular part of the symmetric matrix, also stored in column major.
        if (row_index != col_index) {
          decision_variable_matrix(col_index, row_index) =
              decision_variable_matrix(row_index, col_index);
        }
        if (row_index + 1 < rows) {
          ++row_index;
        } else {
          ++col_index;
          row_index = col_index;
        }
      }
    }

    AppendNanToEnd(num_new_vars, &x_initial_guess_);
  }

  MatrixXDecisionVariable NewVariables(VarType type, int rows, int cols,
                                       bool is_symmetric,
                                       const std::vector<std::string>& names);

  template <typename T>
  void NewIndeterminates_impl(
      const T& names, Eigen::Ref<MatrixXIndeterminate> indeterminates_matrix) {
    const int rows = indeterminates_matrix.rows();
    const int cols = indeterminates_matrix.cols();
    const int num_new_vars = rows * cols;

    DRAKE_ASSERT(static_cast<int>(names.size()) == num_new_vars);
    int row_index = 0;
    int col_index = 0;
    for (int i = 0; i < num_new_vars; ++i) {
      indeterminates_.emplace_back(names[i]);

      const int new_var_index = indeterminates_.size() - 1;
      indeterminates_index_.insert(std::make_pair(
          indeterminates_[new_var_index].get_id(), new_var_index));
      indeterminates_matrix(row_index, col_index) =
          indeterminates_[new_var_index];

      // store the indeterminate in column major.
      if (row_index + 1 < rows) {
        ++row_index;
      } else {
        ++col_index;
        row_index = 0;
      }
    }
  }

  /*
   * Given a matrix of decision variables, checks if every entry in the
   * matrix is a decision variable in the program; throws a runtime
   * error if any variable is not a decision variable in the program.
   * @param vars A vector of variables.
   */
  void CheckIsDecisionVariable(const VectorXDecisionVariable& vars) const;

  /*
   * Ensure a binding is valid *before* adding it to the program.
   * @pre The binding has not yet been registered.
   * @pre The decision variables have been registered.
   * @throws std::exception if the binding is invalid.
   * @returns true if the binding is non-trivial (>= 1 output); when false,
   *   this program should avoid adding the binding to its internal state.
   */
  template <typename C>
  [[nodiscard]] bool CheckBinding(const Binding<C>& binding) const;

  /*
   * Adds new variables to MathematicalProgram.
   */
  template <int Rows, int Cols>
  MatrixDecisionVariable<Rows, Cols> NewVariables(
      VarType type, const typename NewVariableNames<Rows, Cols>::type& names,
      int rows, int cols) {
    DRAKE_DEMAND(rows >= 0 && cols >= 0);
    MatrixDecisionVariable<Rows, Cols> decision_variable_matrix;
    decision_variable_matrix.resize(rows, cols);
    NewVariables_impl(type, names, false, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /*
   * Adds symmetric matrix variables to optimization program. Only the lower
   * triangular part of the matrix is used as decision variables.
   * @param names The names of the stacked columns of the lower triangular part
   * of the matrix.
   */
  template <int Rows>
  MatrixDecisionVariable<Rows, Rows> NewSymmetricVariables(
      VarType type, const typename NewSymmetricVariableNames<Rows>::type& names,
      int rows = Rows) {
    MatrixDecisionVariable<Rows, Rows> decision_variable_matrix(rows, rows);
    NewVariables_impl(type, names, true, decision_variable_matrix);
    return decision_variable_matrix;
  }

  /*
   * Create a new free polynomial, and add its coefficients as decision
   * variables.
   */
  symbolic::Polynomial NewFreePolynomialImpl(
      const symbolic::Variables& indeterminates, int degree,
      const std::string& coeff_name,
      // TODO(jwnimmer-tri) Fix this to not depend on all of "monomial_util.h"
      // for just this tiny enum (e.g., use a bare int == 0,1,2 instead).
      symbolic::internal::DegreeType degree_type);

  void CheckVariableType(VarType var_type);

  // maps the ID of a symbolic variable to the index of the variable stored
  // in the optimization program.
  std::unordered_map<symbolic::Variable::Id, int> decision_variable_index_{};

  // Use std::vector here instead of Eigen::VectorX because std::vector performs
  // much better when pushing new variables into the container.
  std::vector<symbolic::Variable> decision_variables_;

  std::unordered_map<symbolic::Variable::Id, int> indeterminates_index_;
  // Use std::vector here instead of Eigen::VectorX because std::vector performs
  // much better when pushing new variables into the container.
  std::vector<symbolic::Variable> indeterminates_;

  std::vector<Binding<VisualizationCallback>> visualization_callbacks_;

  std::vector<Binding<Cost>> generic_costs_;
  std::vector<Binding<QuadraticCost>> quadratic_costs_;
  std::vector<Binding<LinearCost>> linear_costs_;
  std::vector<Binding<L2NormCost>> l2norm_costs_;

  // note: linear_constraints_ does not include linear_equality_constraints_
  std::vector<Binding<Constraint>> generic_constraints_;
  std::vector<Binding<LinearConstraint>> linear_constraints_;
  std::vector<Binding<LinearEqualityConstraint>> linear_equality_constraints_;
  std::vector<Binding<BoundingBoxConstraint>> bbox_constraints_;
  std::vector<Binding<QuadraticConstraint>> quadratic_constraints_;
  std::vector<Binding<LorentzConeConstraint>> lorentz_cone_constraint_;
  std::vector<Binding<RotatedLorentzConeConstraint>>
      rotated_lorentz_cone_constraint_;
  std::vector<Binding<PositiveSemidefiniteConstraint>>
      positive_semidefinite_constraint_;
  std::vector<Binding<LinearMatrixInequalityConstraint>>
      linear_matrix_inequality_constraint_;
  std::vector<Binding<ExponentialConeConstraint>> exponential_cone_constraints_;

  // Invariant:  The bindings in this list must be non-overlapping, when calling
  // Linear Complementarity solver like Moby. If this constraint is solved
  // through a nonlinear optimization solver (like SNOPT) instead, then we allow
  // the bindings to be overlapping.
  // TODO(ggould-tri) can this constraint be relaxed?
  std::vector<Binding<LinearComplementarityConstraint>>
      linear_complementarity_constraints_;

  Eigen::VectorXd x_initial_guess_;

  // The actual per-solver customization options.
  SolverOptions solver_options_;

  ProgramAttributes required_capabilities_;

  std::unordered_map<int, double> var_scaling_map_{};
};

std::ostream& operator<<(std::ostream& os, const MathematicalProgram& prog);

}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, MathematicalProgram, x, x.to_string())
