#include "drake/common/symbolic_environment.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace drake {
namespace symbolic {

using std::endl;
using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::string;

Environment::Environment(std::initializer_list<value_type> init) : map_(init) {
  for (const auto& p : init) {
    if (std::isnan(p.second)) {
      ostringstream oss;
      oss << "(" << p.first << ", " << p.second << ")"
          << " is detected in the initialization of a symbolic environment.";
      throw runtime_error(oss.str());
    }
  }
}

void Environment::insert(const key_type& key, const mapped_type& elem) {
  map_.emplace(key, elem);
}

string Environment::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

ostream& operator<<(ostream& os, const Environment& env) {
  for (const auto& p : env) {
    os << p.first << " -> " << p.second << endl;
  }
  return os;
}
}  // namespace symbolic
}  // namespace drake
