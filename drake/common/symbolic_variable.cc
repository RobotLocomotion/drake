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

Variable::Variable(std::string const& name) : id_(next_id_++), name_(name) {}
size_t Variable::get_id() const { return id_; }
std::string Variable::get_name() const { return name_; }
bool operator<(Variable const& lhs, Variable const& rhs) {
  return lhs.get_id() < rhs.get_id();
}
bool operator==(Variable const& lhs, Variable const& rhs) {
  return lhs.get_id() == rhs.get_id();
}

ostream& operator<<(ostream& os, Variable const& var) {
  os << var.get_name();
  return os;
}

}  // namespace symbolic
}  // namespace drake

namespace std {
string to_string(drake::symbolic::Variable const& v) {
  ostringstream oss;
  oss << v;
  return oss.str();
}
}  // namespace std
