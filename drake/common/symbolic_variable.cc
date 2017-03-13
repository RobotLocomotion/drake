#include "drake/common/symbolic_variable.h"

#include <atomic>
#include <iostream>
#include <sstream>
#include <string>

#include "drake/common/never_destroyed.h"

using std::atomic;
using std::ostream;
using std::ostringstream;
using std::string;

namespace drake {
namespace symbolic {

Variable::Id Variable::get_next_id() {
  // Note that id 0 is reserved for anonymous variable which is created by the
  // default constructor, Variable(). As a result, we have an invariant
  // "get_next_id() > 0".
  static never_destroyed<atomic<Id>> next_id(1);
  return next_id.access()++;
}

Variable::Variable(const string& name) : id_{get_next_id()}, name_{name} {
  DRAKE_ASSERT(id_ > 0);
}
Variable::Id Variable::get_id() const { return id_; }
string Variable::get_name() const { return name_; }
string Variable::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

ostream& operator<<(ostream& os, const Variable& var) {
  os << var.get_name();
  return os;
}

}  // namespace symbolic
}  // namespace drake
