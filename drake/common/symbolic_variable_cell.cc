#include "drake/common/symbolic_variable_cell.h"

#include <cstddef>
#include <functional>
#include <ostream>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/hash.h"

using std::string;
using std::hash;
using std::ostream;

namespace drake {
namespace symbolic {
VariableCell::VariableCell(const VariableKind kind, const string& name)
    : VariableCell{kind, name, 0} {}

VariableCell::VariableCell(const VariableKind kind, const string& name,
                           const size_t hash)
    : kind_{kind},
      name_{name},
      hash_{hash_combine(hash, name_, static_cast<size_t>(kind_))} {}

VariableKind VariableCell::kind() const { return kind_; }
const string& VariableCell::name() const { return name_; }
size_t VariableCell::hash() const { return hash_; }

}  // namespace symbolic
}  // namespace drake
