#include "drake/multibody/tree/scoped_name.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

using std::optional;
using std::string;
using std::string_view;

constexpr string_view kDelim = internal::kScopedNameDelim;

ScopedName::ScopedName(string_view namespace_in, string_view element) {
  optional<ScopedName> joined = Join(namespace_in, element);
  if (!joined) {
    throw std::logic_error(fmt::format(
        "Cannot create a ScopedName using an already-delimited element name"
        " '{}'", element));
  }
  *this = std::move(*joined);
}

optional<ScopedName> ScopedName::Join(
    string_view namespace_in, string_view element) {
  optional<ScopedName> result;
  if (element.find(kDelim) != string_view::npos) {
    return result;
  }
  result = ScopedName{};
  if (namespace_in.empty()) {
    result->name_ = element;
    result->element_begin_ = 0;
  } else {
    result->name_ = fmt::format("{}{}{}", namespace_in, kDelim, element);
    result->element_begin_ = namespace_in.size() + kDelim.size();
  }
  return result;
}

ScopedName ScopedName::Parse(string scoped_name) {
  const size_t delim_begin = scoped_name.rfind(kDelim);
  ScopedName result;
  result.name_ = std::move(scoped_name);
  if (delim_begin != string::npos) {
    result.element_begin_ = delim_begin + kDelim.size();
  } else {
    result.element_begin_ = 0;
  }
  return result;
}

string_view ScopedName::get_namespace() const {
  if (element_begin_ == 0) {
    return string_view{};
  }
  DRAKE_DEMAND(element_begin_ >= 2);
  const size_t count = element_begin_ - 2;
  return string_view{name_}.substr(0, count);
}

string_view ScopedName::get_element() const {
  return string_view{name_}.substr(element_begin_);
}

string_view ScopedName::to_string() const {
  return name_;
}

}  // namespace multibody
}  // namespace drake
