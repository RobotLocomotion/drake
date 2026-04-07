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
  static uint64_t global_hi_tare = 0;
  static uint64_t global_lo_tare = 0;
  std::call_once(flag, []() {
    // We need to draw actual physical entropy from a random_device, so that
    // variable IDs differ from one run of a process to the next.
    std::random_device device;
    std::mt19937_64 generator(device());

    // Draw a random 56-bit number, in the upper bits of a 64-bit word. (We
    // leave the lower 8 bits as zero, to be used for the Variable::Type.)
    using distribution = std::uniform_int_distribution<uint64_t>;
    distribution rand56(0, (uint64_t{1} << 56) - 1);
    global_hi_tare = rand56(generator) << 8;
    DRAKE_ASSERT((global_hi_tare << 56) == 0);

    // Draw a random 32-bit number, in the upper bits of a 64-bit word. (We
    // leave the lower 32 bits as zero, to be used for a consistent hash code
    // based on the serial number.)
    distribution rand32(0, (uint64_t{1} << 32) - 1);
    global_lo_tare = rand32(generator) << 32;
    DRAKE_ASSERT((global_lo_tare << 32) == 0);
  });

  // Each variable created gets a serial number counting up from 1.
  // (Zero is reserved for the default-constructed Id.)
  static atomic<uint64_t> counter(0);
  const uint64_t serial_number = ++counter;

  Id result;
  // We store the Type enum in the lower byte of hi_.
  static_assert(sizeof(Type) == 1);
  result.hi_ = global_hi_tare + static_cast<uint64_t>(type);
  // Once the serial number reaches 2^32, it will leak into the "random" part
  // of lo_ (the upper 32 bits), but that is not a problem. We still guarantee
  // that lo_ is unique across all variables created by this program.
  result.lo_ = global_lo_tare + serial_number;
  return result;
}

std::string Variable::Id::to_string() const {
  // Our string representation is 18+16 characters: "0x" followed by 16+16 hex
  // digits to cover all 128 bits (padded with leading zeros as necessary).
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
