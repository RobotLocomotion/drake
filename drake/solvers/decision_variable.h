#pragma once

#include <cstddef>
#include <initializer_list>
#include <list>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

class DecisionVariableScalar {
 public:
  enum class VarType {CONTINUOUS, INTEGER, BINARY};
  DecisionVariableScalar(VarType type, const std::string& name, size_t index)
      : type_(type), name_(name), value_(0), index_(index) {}

  VarType type() const {return type_;}

  const std::string& name() const {return name_;}
  double value() const {return value_;}
  size_t index() const {return index_;}

  void set_value(double new_value) {value_ = new_value;}
 private:
  VarType type_;
  std::string name_;
  double value_;
  size_t index_;
};

/**
 * A matrix of decision variables. This class stores the decision variables.
 * The user can access the values of the decision variables through value()
 * function, after calling Solve() in the MathematicalProgram or
 * MathematicalProgramSolverInterface
 */
class MatrixDecisionVariable {
 public:
  MatrixDecisionVariable(size_t rows, size_t cols, const std::vector<std::reference_wrapper<const DecisionVariableScalar>>& vars) :
      rows_(rows), cols_(cols), is_symmetric_(false), vars_(vars.begin(), vars.end()){
    DRAKE_ASSERT(rows * cols == vars.size());
  }
  MatrixDecisionVariable operator()(int i, int j) const {
    return MatrixDecisionVariable(1, 1, {vars_[MatrixIndicesToVectorIndex(i, j)]});
  }

  double value(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().value();
  }

  DecisionVariableScalar::VarType type(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().type();
  }

  const std::string& name(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().name();
  }

  size_t index(size_t i, size_t j) const {
    return vars_[MatrixIndicesToVectorIndex(i, j)].get().index();
  }

  bool is_symmetric() const {return is_symmetric_;}

  /**
   * Return the value of all the decision variables stored inside the class
   */
  Eigen::MatrixXd value() const {
    Eigen::MatrixXd mat(rows_, cols_);
    for (int i = 0; i < static_cast<int>(rows_); ++i) {
      for (int j = 0; j < static_cast<int>(cols_); ++j) {
        mat(i, j) = vars_[MatrixIndicesToVectorIndex(i, j)].get().value();
      }
    }
    return mat;
  }

  MatrixDecisionVariable block(size_t row_start, size_t col_start, size_t rows, size_t cols) const {
    DRAKE_ASSERT(row_start + rows <= rows_ && col_start + cols <= cols_);
    std::vector<std::reference_wrapper<const DecisionVariableScalar>> vars;
    vars.reserve(rows * cols);
    for (int j = 0; j < static_cast<int>(cols); ++j) {
      for (int i = 0; i < static_cast<int>(rows); ++i) {
        vars.push_back(vars_[MatrixIndicesToVectorIndex(row_start + i, col_start + j)]);
      }
    }
    return MatrixDecisionVariable(rows, cols, vars);
  }

 private:
  size_t rows_;
  size_t cols_;
  bool is_symmetric_;
  std::vector<std::reference_wrapper<const DecisionVariableScalar>> vars_;
  /**
   * Return the index in vars_ given matrix index (i, j).
   * @param i
   * @param j
   * @return
   */
  int MatrixIndicesToVectorIndex(size_t i, size_t j) const {
    DRAKE_ASSERT(i < rows_ && j < cols_);
    return j * rows_ + i;
  }
};

/**
 * DecisionVariable
 * @brief Provides storage for a decision variable inside an
 * MathematicalProgram.
 */
class DecisionVariable {
 public:
  enum class VarType { CONTINUOUS, INTEGER, BINARY };

  DecisionVariable(VarType type, const std::string& name, size_t num_vars,
                   size_t start_index)
      : type_(type),
        name_(name),
        data_(Eigen::VectorXd::Zero(num_vars)),
        start_index_(start_index) {}

  /** index()
   * @brief returns the first index of this variable in the entire variable
   * vector for the program
   */
  size_t index() const { return start_index_; }
  /** size()
   * @brief returns the number of elements in the decision variable vector
   */
  size_t size() const { return data_.size(); }
  /** name()
   * @return the name of the DecisionVariable
   */
  const std::string& name() const { return name_; }
  /** value()
   * @brief returns the actual stored value; which is only meaningful after
   * calling solve() in the program.
   */
  const Eigen::VectorXd& value() const { return data_; }
  void set_value(const Eigen::VectorXd& new_data) { data_ = new_data; }

  VarType type() const {return type_;}

 private:
  VarType type_;
  std::string name_;
  Eigen::VectorXd data_;
  size_t start_index_;
};

class DecisionVariableView {  // enables users to access pieces of the decision
                              // variables like they would any other eigen
                              // vector
 public:
  /// Create a view which covers an entire DecisionVariable.
  ///
  /// @p var is aliased, and must remain valid for the lifetime of the view.
  explicit DecisionVariableView(const DecisionVariable& var)
      : var_(var), start_index_(0), size_(var_.size()) {}

  /// Create a view covering part of a DecisionVariable.
  ///
  /// @p var is aliased, and must remain valid for the lifetime of the view.
  DecisionVariableView(const DecisionVariable& var, size_t start, size_t n)
      : var_(var), start_index_(start), size_(n) {
    DRAKE_ASSERT((start + n) <= static_cast<size_t>(var.size()));
  }

  /** index()
   * @brief returns the first index of this variable in the entire variable
   * vector for the program
   */
  size_t index() const { return var_.index() + start_index_; }

  /** size()
   * @brief returns the number of elements in the decision variable vector
   */
  size_t size() const { return size_; }

  /** value()
   * @brief returns the actual stored value; which is only meaningful after
   * calling solve() in the program.
   */
  Eigen::VectorBlock<const Eigen::VectorXd, Eigen::Dynamic> value() const {
    return var_.value().segment(start_index_, size_);
  }

  std::string name() const {
    if ((start_index_ == 0) &&
        (size_ == static_cast<size_t>(var_.value().size()))) {
      return var_.name();
    } else {
      return var_.name() + "(" + std::to_string(start_index_) + ":" +
             std::to_string(start_index_ + size_) + ")";
    }
  }

  /** covers()
   * @brief returns true iff the given @p index of the enclosing
   * MathematicalProgram is included in this VariableView.*/
  bool covers(size_t var_index) const {
    return (var_index >= index()) && (var_index < (index() + size_));
  }

  const DecisionVariableView operator()(size_t i) const {
    DRAKE_ASSERT(i <= size_);
    return DecisionVariableView(var_, start_index_ + i, 1);
  }
  const DecisionVariableView row(size_t i) const {
    DRAKE_ASSERT(i <= size_);
    return DecisionVariableView(var_, start_index_ + i, 1);
  }
  const DecisionVariableView head(size_t n) const {
    DRAKE_ASSERT(n <= size_);
    return DecisionVariableView(var_, start_index_, n);
  }
  const DecisionVariableView tail(size_t n) const {
    DRAKE_ASSERT(n <= size_);
    return DecisionVariableView(var_, start_index_ + size_ - n, n);
  }
  const DecisionVariableView segment(size_t start, size_t n) const {
    DRAKE_ASSERT(start + n <= size_);
    return DecisionVariableView(var_, start_index_ + start, n);
  }

 private:
  const DecisionVariable& var_;
  size_t start_index_, size_;
};

typedef std::list<DecisionVariableView> VariableList;
inline int GetVariableListSize(const VariableList& vars) {
  int var_dim = 0;
  for (const auto& var : vars) {
    var_dim += var.size();
  }
  return var_dim;
}

}  // end namespace solvers
}  // end namespace drake
