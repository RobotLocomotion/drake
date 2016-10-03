#include "drake/common/symbolic_environment.h"

#include <iostream>
#include <sstream>

#include "drake/common/drake_assert.h"

namespace drake {
namespace symbolic {

using std::endl;
using std::ostream;
using std::ostringstream;

Environment::Environment(std::initializer_list<value_type> init) : map_(init) {}

void Environment::insert(key_type const & key, mapped_type const & elem) {
  map_.emplace(key, elem);
}

ostream& operator<<(ostream& os, symbolic::Environment const& env) {
  for (auto const& p : env) {
    os << p.first << " -> " << p.second << endl;
  }
  return os;
}
}  // namespace symbolic
}  // namespace drake

namespace std {
string to_string(drake::symbolic::Environment const& env) {
  ostringstream oss;
  oss << env;
  return oss.str();
}
}  // namespace std
