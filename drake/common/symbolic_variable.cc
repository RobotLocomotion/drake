#include "drake/common/symbolic_variable.h"

#include <atomic>
#include <iostream>
#include <sstream>
#include <string>

using std::atomic;
using std::ostream;
using std::ostringstream;
using std::string;

namespace drake {
namespace symbolic {

size_t Variable::get_next_id() {
  // Purposefully never freed to avoid static initialization fiasco.
  static atomic<size_t>* next_id = new atomic<size_t>{0};
  return (*next_id)++;
}

Variable::Variable(const string& name) : id_(get_next_id()), name_(name) {}
size_t Variable::get_id() const { return id_; }
string Variable::get_name() const { return name_; }
string Variable::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}
bool operator<(const Variable& lhs, const Variable& rhs) {
  return lhs.get_id() < rhs.get_id();
}
bool operator==(const Variable& lhs, const Variable& rhs) {
  return lhs.get_id() == rhs.get_id();
}

ostream& operator<<(ostream& os, const Variable& var) {
  os << var.get_name();
  return os;
}

}  // namespace symbolic
}  // namespace drake
