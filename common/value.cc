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
  const std::string message = fmt::format(
      "TypeHash<T> cannot operate on T={}; Value<T> may suffer from slightly"
      " impaired performance. If T uses a single non-type template parameter,"
      " adding 'using NonTypeTemplateParameter = ...;' will enable hashing."
      " See drake/common/test/value_test.cc for an example.",
      NiceTypeName::Get(detail));
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
  throw std::logic_error(fmt::format(
      "AbstractValue: a request to cast to '{}' failed because "
      "the actual type was '{}'.", requested_type, GetNiceTypeName()));
}

}  // namespace drake
