/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include): Our header file is included by all.h.
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

using std::atomic;
using std::make_shared;
using std::ostream;
using std::string;

namespace drake {
namespace symbolic {

Variable::Id Variable::Id::Create(Type type) {
  // Each variable created gets a serial number counting up from 1.
  // (Zero is reserved for the default-constructed Id.)
  static atomic<uint64_t> counter(0);
  const uint64_t serial_number = ++counter;

  Id result;
  // We store the Type enum in the upper byte (8 bits) of value_ (64 bits).
  static_assert(sizeof(Type) == 1);
  result.value_ = (static_cast<uint64_t>(type) << (64 - 8)) + serial_number;
  return result;
}

std::string Variable::Id::to_string() const {
  // Our string repsentation is 18 characters: "0x" followed by 16 hex digits to
  // cover all 64 bits (padded with leading zeros as necessary).
  return fmt::format("{:#018x}", value_);
}

Variable::Variable(string name, const Type type)
    : id_{Id::Create(type)}, name_{make_shared<const string>(std::move(name))} {
  DRAKE_ASSERT(get_id().value() > 0);
}

string Variable::get_name() const {
  return name_ != nullptr ? *name_ : string{"𝑥"};
}

string Variable::to_string() const {
  return get_name();
}

ostream& operator<<(ostream& os, const Variable& var) {
  os << var.get_name();
  return os;
}

ostream& operator<<(ostream& os, Variable::Type type) {
  return os << fmt::to_string(type);
}

std::string_view to_string(const Variable::Type& type) {
  switch (type) {
    case Variable::Type::CONTINUOUS:
      return "Continuous";
    case Variable::Type::BINARY:
      return "Binary";
    case Variable::Type::INTEGER:
      return "Integer";
    case Variable::Type::BOOLEAN:
      return "Boolean";
    case Variable::Type::RANDOM_UNIFORM:
      return "Random Uniform";
    case Variable::Type::RANDOM_GAUSSIAN:
      return "Random Gaussian";
    case Variable::Type::RANDOM_EXPONENTIAL:
      return "Random Exponential";
  }
  DRAKE_UNREACHABLE();
}

MatrixX<Variable> MakeMatrixVariable(const int rows, const int cols,
                                     const string& name,
                                     const Variable::Type type) {
  MatrixX<Variable> m{rows, cols};
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      m(i, j) = Variable{fmt::format("{}({}, {})", name, i, j), type};
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
    vec[i] = Variable{fmt::format("{}({})", name, i), type};
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
