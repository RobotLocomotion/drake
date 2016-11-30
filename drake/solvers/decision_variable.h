#pragma once

#include <cstddef>

#include <list>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

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
