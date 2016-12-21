#include "drake/common/symbolic_variable.h"

#include <memory>
#include <ostream>
#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_indeterminate.h"
#include "drake/common/symbolic_variable_cell.h"

using std::ostream;
using std::ostringstream;
using std::string;
using std::make_shared;
using std::shared_ptr;

namespace drake {
namespace symbolic {

/** Total ordering between VariableKind. */
bool operator<(const VariableKind k1, const VariableKind k2) {
  return static_cast<size_t>(k1) < static_cast<size_t>(k2);
}

Variable::Variable(const std::string& name)
    : Variable{make_indeterminate(name)} {}
Variable::Variable(const shared_ptr<VariableCell> ptr) : ptr_{ptr} {}

VariableKind Variable::kind() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->kind();
}

string Variable::name() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->name();
}

string Variable::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

size_t Variable::hash() const {
  DRAKE_ASSERT(ptr_ != nullptr);
  return ptr_->hash();
}

bool Variable::equal_to(const Variable& var) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_ASSERT(var.ptr_ != nullptr);
  if (ptr_ == var.ptr_) {
    return true;
  }
  if (kind() != var.kind()) {
    return false;
  }
  if (hash() != var.hash()) {
    return false;
  }
  // Same kind/hash, but it could be the result of hash collision,
  // Check cell equality.
  return ptr_->equal_to(*(var.ptr_));
}

bool Variable::less(const Variable& var) const {
  DRAKE_ASSERT(ptr_ != nullptr);
  DRAKE_ASSERT(var.ptr_ != nullptr);
  if (ptr_ == var.ptr_) {
    return false;  // this equals to e, not less-than.
  }
  const VariableKind k1{kind()};
  const VariableKind k2{var.kind()};
  if (k1 < k2) {
    return true;
  }
  if (k2 < k1) {
    return false;
  }
  // k1 == k2
  return ptr_->less(*(var.ptr_));
}

ostream& operator<<(ostream& os, const Variable& var) {
  DRAKE_ASSERT(var.ptr_ != nullptr);
  return var.ptr_->Display(os);
}

bool operator<(const Variable& lhs, const Variable& rhs) {
  return lhs.less(rhs);
}

bool operator==(const Variable& lhs, const Variable& rhs) {
  return lhs.equal_to(rhs);
}

Variable make_indeterminate(const string& name) {
  return Variable{make_shared<Indeterminate>(name)};
}

}  // namespace symbolic
}  // namespace drake
