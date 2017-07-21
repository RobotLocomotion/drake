// NOLINTNEXTLINE(build/include): Its header file is included in symbolic.h.
#include <atomic>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/symbolic.h"

using std::atomic;
using std::make_shared;
using std::move;
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

Variable::Variable(string name, const Type type)
    : id_{get_next_id()}, type_{type}, name_{make_shared<string>(move(name))} {
  DRAKE_ASSERT(id_ > 0);
}
Variable::Id Variable::get_id() const { return id_; }
Variable::Type Variable::get_type() const { return type_; }
string Variable::get_name() const { return *name_; }
string Variable::to_string() const {
  ostringstream oss;
  oss << *this;
  return oss.str();
}

ostream& operator<<(ostream& os, const Variable& var) {
  os << var.get_name();
  return os;
}

ostream& operator<<(ostream& os, Variable::Type type) {
  switch (type) {
    case Variable::Type::CONTINUOUS:
      return os << "Continuous";
    case Variable::Type::BINARY:
      return os << "Binary";
    case Variable::Type::INTEGER:
      return os << "Integer";
    case Variable::Type::BOOLEAN:
      return os << "Boolean";
  }
  // Should be unreachable.
  DRAKE_ABORT();
}
}  // namespace symbolic
}  // namespace drake
