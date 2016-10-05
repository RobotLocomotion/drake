#include "drake/common/symbolic_variable.h"

#include <iostream>
#include <sstream>
#include <string>

#include "drake/common/drake_assert.h"

using std::string;
using std::ostream;
using std::ostringstream;

namespace drake {
namespace symbolic {

size_t Variable::next_id_{};

Variable::Variable(const std::string& name) : id_(next_id_++), name_(name) {}
size_t Variable::get_id() const { return id_; }
std::string Variable::get_name() const { return name_; }
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

namespace std {
string to_string(const drake::symbolic::Variable& v) {
  ostringstream oss;
  oss << v;
  return oss.str();
}
}  // namespace std
