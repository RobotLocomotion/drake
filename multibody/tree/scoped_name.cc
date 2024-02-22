#include "drake/multibody/tree/scoped_name.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

namespace {

constexpr std::string_view kDelim{"::"};
constexpr std::string_view kRepeatedDelim{"::::"};

bool HasScopeDelimiter(std::string_view name) {
  return name.find(kDelim) != std::string_view::npos;
}

}  // namespace

ScopedName::ScopedName(std::string_view namespace_name,
                       std::string_view element_name) {
  std::optional<ScopedName> result = Make(namespace_name, element_name);
  if (!result) {
    throw std::logic_error(fmt::format(
        "Cannot create a ScopedName('{}', '{}') with mis-placed delimiters",
        namespace_name, element_name));
  }
  *this = std::move(*result);
}

std::optional<ScopedName> ScopedName::Make(std::string_view namespace_name,
                                           std::string_view element_name) {
  if (namespace_name.starts_with(kDelim) || namespace_name.ends_with(kDelim) ||
      element_name.empty() || HasScopeDelimiter(element_name)) {
    return std::nullopt;
  }
  ScopedName result;
  if (namespace_name.empty()) {
    result.name_ = element_name;
    result.element_begin_ = 0;
  } else {
    result.name_ = fmt::format("{}{}{}", namespace_name, kDelim, element_name);
    result.element_begin_ = namespace_name.size() + kDelim.size();
  }
  return result;
}

ScopedName ScopedName::Join(std::string_view name1, std::string_view name2) {
  return Parse(fmt::format("{}{}{}", name1, kDelim, name2));
}

ScopedName ScopedName::Parse(std::string scoped_name) {
  // Remove any leading or trailing "::".
  while (scoped_name.ends_with(kDelim)) {
    scoped_name.resize(scoped_name.size() - kDelim.size());
  }
  while (scoped_name.starts_with(kDelim)) {
    scoped_name = scoped_name.substr(kDelim.size());
  }

  // De-duplicate any interior "::::" -> "::".
  while (true) {
    const size_t repeat_begin = scoped_name.rfind(kRepeatedDelim);
    if (repeat_begin == std::string::npos) {
      break;
    }
    scoped_name.replace(repeat_begin, kRepeatedDelim.size(), kDelim);
  }

  // Now that the string is canonical, create out result.
  ScopedName result;
  result.name_ = std::move(scoped_name);
  const size_t delim_begin = result.name_.rfind(kDelim);
  if (delim_begin != std::string::npos) {
    result.element_begin_ = delim_begin + kDelim.size();
  } else {
    result.element_begin_ = 0;
  }
  return result;
}

std::string_view ScopedName::get_namespace() const {
  if (element_begin_ == 0) {
    return std::string_view{};
  }
  DRAKE_DEMAND(element_begin_ >= kDelim.size());
  const size_t count = element_begin_ - kDelim.size();
  return std::string_view{name_}.substr(0, count);
}

std::string_view ScopedName::get_element() const {
  return std::string_view{name_}.substr(element_begin_);
}

std::string_view ScopedName::get_full() const {
  return name_;
}

std::string ScopedName::to_string() const {
  return name_;
}

void ScopedName::set_namespace(std::string_view namespace_name) {
  *this = ScopedName(namespace_name, get_element());
}

void ScopedName::set_element(std::string_view element_name) {
  *this = ScopedName(get_namespace(), element_name);
}

}  // namespace multibody
}  // namespace drake
