// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

using std::endl;
using std::initializer_list;
using std::move;
using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::string;

namespace {
void throw_if_dummy(const Variable& var) {
  if (var.is_dummy()) {
    ostringstream oss;
    oss << "Dummy variable (ID = 0) is detected"
        << "in the initialization of an environment.";
    throw runtime_error(oss.str());
  }
}

void throw_if_nan(const double v) {
  if (std::isnan(v)) {
    ostringstream oss;
    oss << "NaN is detected in the initialization of an environment.";
    throw runtime_error(oss.str());
  }
}

// Given a list of variables, @p vars, builds an Environment::map which maps a
// Variable to its double value. All values are set to 0.0.
Environment::map BuildMap(const initializer_list<Environment::key_type> vars) {
  Environment::map m;
  for (const Environment::key_type& var : vars) {
    m.emplace(var, 0.0);
  }
  return m;
}

}  // anonymous namespace

Environment::Environment(const std::initializer_list<value_type> init)
    : Environment{map(init)} {}

Environment::Environment(const std::initializer_list<key_type> vars)
    : Environment{BuildMap(vars)} {}

Environment::Environment(map m) : map_{move(m)} {
  for (const auto& p : map_) {
    throw_if_dummy(p.first);
    throw_if_nan(p.second);
  }
}

void Environment::insert(const key_type& key, const mapped_type& elem) {
  throw_if_dummy(key);
  throw_if_nan(elem);
  map_.emplace(key, elem);
}

Variables Environment::domain() const {
  Variables dom;
  for (const auto& p : map_) {
    dom += p.first;
  }
  return dom;
}

string Environment::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

Environment::mapped_type& Environment::operator[](const key_type& key) {
  if (key.is_dummy()) {
    ostringstream oss;
    oss << "Environment::operator[] is called with a dummy variable.";
    throw runtime_error(oss.str());
  }
  return map_[key];
}

const Environment::mapped_type& Environment::operator[](
    const key_type& key) const {
  if (key.is_dummy()) {
    ostringstream oss;
    oss << "Environment::operator[] is called with a dummy variable.";
    throw runtime_error(oss.str());
  }
  if (!map_.count(key)) {
    ostringstream oss;
    oss << "Environment::operator[] was called on a const Environment "
        << "with a missing key \"" << key << "\".";
    throw runtime_error(oss.str());
  }
  return map_.at(key);
}

ostream& operator<<(ostream& os, const Environment& env) {
  for (const auto& p : env) {
    os << p.first << " -> " << p.second << endl;
  }
  return os;
}
}  // namespace symbolic
}  // namespace drake
