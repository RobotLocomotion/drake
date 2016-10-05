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

void Environment::insert(const key_type& key, const mapped_type& elem) {
  map_.emplace(key, elem);
}

ostream& operator<<(ostream& os, const Environment& env) {
  for (const auto& p : env) {
    os << p.first << " -> " << p.second << endl;
  }
  return os;
}
}  // namespace symbolic
}  // namespace drake

namespace std {
string to_string(const drake::symbolic::Environment& env) {
  ostringstream oss;
  oss << env;
  return oss.str();
}
}  // namespace std
