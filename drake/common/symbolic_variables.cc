#include "drake/common/symbolic_variables.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "drake/common/drake_assert.h"
#include "drake/common/hash_combine.h"

using std::accumulate;
using std::includes;
using std::string;
using std::ostringstream;
using std::ostream_iterator;
using std::ostream;

namespace drake {
namespace symbolic {

Variables::Variables(std::initializer_list<value_type> init) : vars_(init) {}

size_t Variables::get_hash() const {
  // combine hashes of the variables in a set
  return accumulate(vars_.begin(), vars_.end(), 0,
                    [](size_t const h, Variable const& var) {
                      return hash_combine(h, var.get_hash());
                    });
}

Variables::size_type Variables::erase(Variables const& vars) {
  size_type num_of_erased_elements{0};
  for (Variable const& var : vars) {
    num_of_erased_elements += erase(var);
  }
  return num_of_erased_elements;
}

bool Variables::IsSubsetOf(Variables const& vars) const {
  return includes(vars.begin(), vars.end(), begin(), end());
}

bool Variables::IsSupersetOf(Variables const& vars) const {
  return vars.IsSubsetOf(*this);
}

bool Variables::IsStrictSubsetOf(Variables const& vars) const {
  if (*this == vars) {
    return false;
  }
  return IsSubsetOf(vars);
}

bool Variables::IsStrictSupersetOf(Variables const& vars) const {
  if (*this == vars) {
    return false;
  }
  return IsSupersetOf(vars);
}

bool operator==(Variables const& vars1, Variables const& vars2) {
  return vars1.vars_ == vars2.vars_;
}

Variables operator+=(Variables& vars1, Variables const& vars2) {
  vars1.insert(vars2);
  return vars1;
}

Variables operator+=(Variables& vars, Variable const& var) {
  vars.insert(var);
  return vars;
}

Variables operator+(Variables vars1, Variables const& vars2) {
  vars1 += vars2;
  return vars1;
}

Variables operator+(Variables vars, Variable const& var) {
  vars += var;
  return vars;
}

Variables operator+(Variable const& var, Variables vars) {
  vars += var;
  return vars;
}

Variables operator-=(Variables& vars1, Variables const& vars2) {
  vars1.erase(vars2);
  return vars1;
}
Variables operator-=(Variables& vars, Variable const& var) {
  vars.erase(var);
  return vars;
}
Variables operator-(Variables vars1, Variables const& vars2) {
  vars1 -= vars2;
  return vars1;
}

Variables operator-(Variables vars, Variable const& var) {
  vars -= var;
  return vars;
}

ostream& operator<<(ostream& os, symbolic::Variables const& vars) {
  os << "{";
  if (!vars.vars_.empty()) {
    // output 1st ... N-1th elements by adding ", " at the end
    copy(vars.begin(), prev(vars.end()), ostream_iterator<Variable>(os, ", "));
    // output the last one (without ",").
    os << *(vars.rbegin());
  }
  os << "}";
  return os;
}

}  // namespace symbolic
}  // namespace drake

namespace std {
string to_string(drake::symbolic::Variables const& vars) {
  ostringstream oss;
  oss << vars;
  return oss.str();
}
}  // namespace std
