#pragma once

#include <cstddef>
#include <functional>
#include <ostream>
#include <string>

#include "drake/common/hash.h"
#include "drake/common/number_traits.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variable_cell.h"

#include <Eigen/Core>

namespace drake {

namespace solvers {
// Forward declaration of drake::solvers::MathematicalProgram. We have it here
// because of the two reasons:
// 1. To avoid mutual dependencies between drake/common:symbolic and
//    drake/solvers:mathematical_program.
// 2. To specify drake::solvers::MathematicalProgram as a friend class of
//    drake::symbolic::DecisionVariableScalar which is defined below.
class MathematicalProgram;  // In drake/solvers/mathematical_program.h
}  // namespace solvers

namespace symbolic {
/**
 * This class stores the type, name, value, and index of a decision variable in
 * an optimization program. The DecisionVariable created by MathematicalProgram
 * should not outlive its creator MathematicalProgram object.
 */
class DecisionVariableScalar : public VariableCell {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  /**
   * This constructor creates a dummy placeholder, the value_ pointer
   * is initialized to nullptr. The usage of this function is
   * @code{.cc}
   * // Creates an optimization program object with no decision variables.
   * MathematicalProgram prog;
   *
   * // Add a 2 x 1 vector containing two decision variables to the optimization
   * // program.
   * // This calls the private constructor
   * // DecisionVariableScalar(VarType type, const std::string &name, double*
   * // value, size_t index)
   * DecisionVariableVector<2> x1 = prog.AddContinuousVariables<2>();
   *
   * // Add a 2 x 1 vector containing two decision variables to the optimization
   * // program.
   * // This calls the private constructor
   * // DecisionVariable(VarType type, const std::string &name, double*
   * // value, size_t index)
   * DecisionVariableVector<2> x2 = prog.AddContinuousVariables<2>();
   *
   * // This calls the default constructor DecisionVariableScalar(),
   * // X is not related to the optimization program prog yet.
   * DecisionVariableMatrix<2, 2> X;
   *
   * // Now X contains the decision variables from the optimization program
   * // object prog.
   * // The first column of X is x1, the second column of X is x2.
   * X << x1, x2;
   * @endcode
   */
  DecisionVariableScalar()
      : DecisionVariableScalar{VarType::CONTINUOUS, "", nullptr, 0} {}

  /** Checks equality. */
  bool equal_to(const VariableCell& var) const override;
  /** Checks total ordering. */
  bool less(const VariableCell& var) const override;
  /** Outputs its string representation to @p os. */
  std::ostream& Display(std::ostream& os) const override;

  /** @return The type of the variable. */
  VarType type() const { return type_; }

  /**
   * @return The value of the variable. This method is only meaningful after
   * calling Solve() in MathematicalProgram.
   */
  double value() const {
    // TODO(hongkai.dai): check if Solve() has been called.
    return *value_;
  }

  /**
   * @return The index of the variable in the optimization program.
   */
  size_t index() const { return index_; }

  /**
   * Determines if the two DecisionVariableScalar objects are the same. This
   * comparison is only meaningful if the two DecisionVariableScalar objects
   * are created by the same MathematicalProgram object.
   */
  bool operator==(const DecisionVariableScalar& rhs) const;

  friend class solvers::MathematicalProgram;

 private:
  /*
   * Constructs a decision variable. We make this constructor private so that
   * only the friend class (aka MathematicalProgram) can construct a decision
   * variable.
   * @param type Supports CONTINUOUS, INTEGER or BINARY.
   * @param name The name of the variable.
   * @param index The index of the variable in the optimization program.
   */
  DecisionVariableScalar(VarType type, const std::string& name, double* value,
                         size_t index)
      : VariableCell{VariableKind::DecisionVariable, name,
                     hash_combine(
                         std::hash<size_t>{}(static_cast<size_t>(type)), value,
                         index)},
        type_{type},
        value_{value},
        index_{index} {}

  void set_value(double new_value) { *value_ = new_value; }
  VarType type_;
  double* value_;
  size_t index_;
};

/**
 * Prints out the DecisionVariableScalar's name.
 * @relates DecisionVariableScalar.
 */
std::ostream& operator<<(std::ostream& os, const DecisionVariableScalar& var);

}  // namespace symbolic
}  // namespace drake

namespace Eigen {
/// Eigen scalar type traits for Matrix<DecisionVariableScalar>.
template <>

struct NumTraits<drake::symbolic::DecisionVariableScalar> {
  static inline int digits10() { return 0; }
  enum {
    IsInteger = 0,
    IsSigned = 1,
    IsComplex = 0,
    RequireInitialization = 1,
    ReadCost = 1,
    AddCost = 1,
    MulCost = 1
  };

  template <bool Vectorized>
  struct Div {
    enum { Cost = 1 };
  };

  typedef drake::symbolic::DecisionVariableScalar Real;
  typedef drake::symbolic::DecisionVariableScalar Nested;
  typedef drake::symbolic::DecisionVariableScalar Literal;
};
}  // namespace Eigen

namespace drake {
template <>
struct is_numeric<symbolic::DecisionVariableScalar> {
  static constexpr bool value = false;
};
}  // namespace drake
