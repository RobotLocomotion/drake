// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <algorithm>
#include <functional>
#include <iterator>
#include <numeric>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

using std::includes;
using std::initializer_list;
using std::inserter;
using std::less;
using std::move;
using std::ostream;
using std::ostream_iterator;
using std::ostringstream;
using std::set;
using std::set_intersection;
using std::string;

namespace drake {
namespace symbolic {

Variables::Variables(std::initializer_list<Variable> init) : vars_(init) {}

Variables::Variables(const Eigen::Ref<const VectorX<Variable>>& vec)
    : vars_{vec.data(), vec.data() + vec.size()} {}

string Variables::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

Variables::size_type Variables::erase(const Variables& vars) {
  size_type num_of_erased_elements{0};
  for (const Variable& var : vars) {
    num_of_erased_elements += erase(var);
  }
  return num_of_erased_elements;
}

bool Variables::IsSubsetOf(const Variables& vars) const {
  return includes(vars.begin(), vars.end(), begin(), end(),
                  std::less<Variable>{});
}

bool Variables::IsSupersetOf(const Variables& vars) const {
  return vars.IsSubsetOf(*this);
}

bool Variables::IsStrictSubsetOf(const Variables& vars) const {
  if (*this == vars) {
    return false;
  }
  return IsSubsetOf(vars);
}

bool Variables::IsStrictSupersetOf(const Variables& vars) const {
  if (*this == vars) {
    return false;
  }
  return IsSupersetOf(vars);
}

bool operator==(const Variables& vars1, const Variables& vars2) {
  return std::equal(vars1.vars_.begin(), vars1.vars_.end(), vars2.vars_.begin(),
                    vars2.vars_.end(), std::equal_to<Variable>{});
}

bool operator<(const Variables& vars1, const Variables& vars2) {
  return std::lexicographical_compare(vars1.vars_.begin(), vars1.vars_.end(),
                                      vars2.vars_.begin(), vars2.vars_.end(),
                                      std::less<Variable>{});
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator+=(Variables& vars1, const Variables& vars2) {
  vars1.insert(vars2);
  return vars1;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator+=(Variables& vars, const Variable& var) {
  vars.insert(var);
  return vars;
}

Variables operator+(Variables vars1, const Variables& vars2) {
  vars1 += vars2;
  return vars1;
}

Variables operator+(Variables vars, const Variable& var) {
  vars += var;
  return vars;
}

Variables operator+(const Variable& var, Variables vars) {
  vars += var;
  return vars;
}

// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator-=(Variables& vars1, const Variables& vars2) {
  vars1.erase(vars2);
  return vars1;
}
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator-=(Variables& vars, const Variable& var) {
  vars.erase(var);
  return vars;
}
Variables operator-(Variables vars1, const Variables& vars2) {
  vars1 -= vars2;
  return vars1;
}

Variables operator-(Variables vars, const Variable& var) {
  vars -= var;
  return vars;
}

Variables::Variables(set<Variable> vars) : vars_{move(vars)} {}

Variables intersect(const Variables& vars1, const Variables& vars2) {
  set<Variable> intersection;
  set_intersection(vars1.vars_.begin(), vars1.vars_.end(), vars2.vars_.begin(),
                   vars2.vars_.end(),
                   inserter(intersection, intersection.begin()),
                   less<Variable>{});
  return Variables{move(intersection)};
}

ostream& operator<<(ostream& os, const Variables& vars) {
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
