/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include): Our header file is included by all.h.
#include "drake/common/symbolic/expression/all.h"
/* clang-format on */

#include <atomic>
#include <memory>
#include <mutex>
#include <random>
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
  static std::once_flag flag;
  static uint64_t hi_tare_ = 0;
  static uint64_t lo_tare_ = 0;
  std::call_once(flag, []() {
    std::random_device device;
    std::mt19937_64 generator(device());

    // Draw a random 56-bit number. (We leave the upper 8 bits as zero, to be
    // used for the Variable::Type.)
    using distribution = std::uniform_int_distribution<uint64_t>;
    distribution rand56(0, (uint64_t{1} << 56) - 1);
    hi_tare_ = rand56(generator);

    // Draw a random 32-bit number, in the upper half of the 64 bits. (We leave
    // the lower 32 bits as zero, to be used for a consistent hash code based on
    // the serial number.)
    distribution rand32(uint64_t{1} << 32, ~uint64_t{0});
    lo_tare_ = rand32(generator);
  });

  // Each variable created fresh in this process gets a serial number.
  static atomic<uint64_t> counter(0);
  const uint64_t serial_number = ++counter;

  Id result;
  result.hi_ = hi_tare_ + (static_cast<uint64_t>(type) << 56);
  result.lo_ = lo_tare_ + serial_number;
  return result;
}

std::string Variable::Id::to_string() const {
  return fmt::format("{:#018x}{:016x}", hi_, lo_);
}

Variable::Variable(string name, const Type type)
    : id_{Id::Create(type)},
      name_{make_shared<const string>(std::move(name))} {}

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
