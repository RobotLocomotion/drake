#include "drake/common/value.h"

#include <atomic>

#include <fmt/format.h>

#include "drake/common/text_logging.h"

namespace drake {

namespace internal {
int ReportZeroHash(const std::type_info& detail) {
  // Log a debug message noting the possible performance impairment.  Elevate
  // it to a warning the first time it happens -- but only elevate it at most
  // once per process, to avoid spamming.
  static std::atomic<bool> g_has_warned{false};
  const bool has_warned = g_has_warned.exchange(true);
  const std::string bad_class = NiceTypeName::Get(detail);
  const std::string message = fmt::format(
      "The {} class is incompatible with the typename hasher that provides the"
      " type-erasure checking for AbstractValue casts, most likely because the"
      " problematic class mixes template parameters with nested classes or"
      " non-type template parameters."
      //
      " As a result, operations on Value<{}> will suffer from slightly impaired"
      " performance."
      //
      " If the problem relates to nested classes, you may be able to resolve it"
      " by un-nesting the class in question."
      //
      " If the problem relates to a single non-type template parameter, you may"
      " be able to resolve it by adding 'using NonTypeTemplateParameter = ...'."
      " See drake/common/test/value_test.cc for an example.",
      bad_class, bad_class);
  if (!has_warned) {
    log()->warn(message +
        " This is the first instance of an impaired T within this process."
        " Additional instances will not be warned about, but you may set"
        " the drake::log() level to 'debug' to see all instances.");
  } else {
    log()->debug(message);
  }
  return 0;
}
}  // namespace internal

AbstractValue::~AbstractValue() = default;

std::string AbstractValue::GetNiceTypeName() const {
  return NiceTypeName::Canonicalize(
      NiceTypeName::Demangle(type_info().name()));
}

void AbstractValue::ThrowCastError(const std::string& requested_type) const {
  const auto dynamic_type = GetNiceTypeName();
  const auto static_type = NiceTypeName::Get(static_type_info());
  if (dynamic_type != static_type) {
    throw std::logic_error(fmt::format(
         "AbstractValue: a request to cast to '{}' failed because the value "
         "was created using the static type '{}' (with a dynamic type of "
         "'{}').", requested_type, static_type, dynamic_type));
  }
  throw std::logic_error(fmt::format(
     "AbstractValue: a request to cast to '{}' failed because the value "
     "was created using the static type '{}'.", requested_type, static_type));
}

}  // namespace drake
