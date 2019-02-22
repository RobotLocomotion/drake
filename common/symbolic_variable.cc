// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <atomic>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"

using std::atomic;
using std::make_shared;
using std::move;
using std::ostream;
using std::ostringstream;
using std::string;
using std::to_string;

namespace drake {
namespace symbolic {

Variable::Id Variable::get_next_id() {
  // Note that id 0 is reserved for anonymous variable which is created by the
  // default constructor, Variable(). As a result, we have an invariant
  // "get_next_id() > 0".
  static never_destroyed<atomic<Id>> next_id(1);
  return next_id.access()++;
}

Variable::Variable(string name, const Type type)
    : id_{get_next_id()},
      type_{type},
      name_{make_shared<const string>(move(name))} {
  DRAKE_ASSERT(id_ > 0);
}
Variable::Id Variable::get_id() const { return id_; }
Variable::Type Variable::get_type() const { return type_; }
string Variable::get_name() const { return *name_; }
string Variable::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

ostream& operator<<(ostream& os, const Variable& var) {
  os << var.get_name();
  return os;
}

ostream& operator<<(ostream& os, Variable::Type type) {
  switch (type) {
    case Variable::Type::CONTINUOUS:
      return os << "Continuous";
    case Variable::Type::BINARY:
      return os << "Binary";
    case Variable::Type::INTEGER:
      return os << "Integer";
    case Variable::Type::BOOLEAN:
      return os << "Boolean";
    case Variable::Type::RANDOM_UNIFORM:
      return os << "Random Uniform";
    case Variable::Type::RANDOM_GAUSSIAN:
      return os << "Random Gaussian";
    case Variable::Type::RANDOM_EXPONENTIAL:
      return os << "Random Exponential";
  }
  DRAKE_UNREACHABLE();
}

MatrixX<Variable> MakeMatrixVariable(const int rows, const int cols,
                                     const string& name,
                                     const Variable::Type type) {
  MatrixX<Variable> m{rows, cols};
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      m(i, j) =
          Variable{name + "(" + to_string(i) + ", " + to_string(j) + ")", type};
    }
  }
  return m;
}

MatrixX<Variable> MakeMatrixBooleanVariable(const int rows, const int cols,
                                            const string& name) {
  return MakeMatrixVariable(rows, cols, name, Variable::Type::BOOLEAN);
}

MatrixX<Variable> MakeMatrixBinaryVariable(const int rows, const int cols,
                                           const string& name) {
  return MakeMatrixVariable(rows, cols, name, Variable::Type::BINARY);
}

MatrixX<Variable> MakeMatrixContinuousVariable(const int rows, const int cols,
                                               const string& name) {
  return MakeMatrixVariable(rows, cols, name, Variable::Type::CONTINUOUS);
}

MatrixX<Variable> MakeMatrixIntegerVariable(const int rows, const int cols,
                                            const string& name) {
  return MakeMatrixVariable(rows, cols, name, Variable::Type::INTEGER);
}

VectorX<Variable> MakeVectorVariable(const int rows, const string& name,
                                     const Variable::Type type) {
  VectorX<Variable> vec{rows};
  for (int i = 0; i < rows; ++i) {
    vec[i] = Variable{name + "(" + to_string(i) + ")", type};
  }
  return vec;
}

VectorX<Variable> MakeVectorBooleanVariable(const int rows,
                                            const string& name) {
  return MakeVectorVariable(rows, name, Variable::Type::BOOLEAN);
}

VectorX<Variable> MakeVectorBinaryVariable(const int rows, const string& name) {
  return MakeVectorVariable(rows, name, Variable::Type::BINARY);
}

VectorX<Variable> MakeVectorContinuousVariable(const int rows,
                                               const string& name) {
  return MakeVectorVariable(rows, name, Variable::Type::CONTINUOUS);
}

VectorX<Variable> MakeVectorIntegerVariable(const int rows,
                                            const string& name) {
  return MakeVectorVariable(rows, name, Variable::Type::INTEGER);
}

}  // namespace symbolic
}  // namespace drake
