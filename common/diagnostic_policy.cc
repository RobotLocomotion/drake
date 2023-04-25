#include "drake/common/diagnostic_policy.h"

#include <sstream>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace internal {

std::string DiagnosticDetail::Format(const std::string& severity) const {
  DRAKE_DEMAND(!severity.empty());
  std::stringstream ss;
  if (filename.has_value()) {
    ss << *filename << ":";
    if (line.has_value()) {
      ss << *line << ":";
    }
    ss << " ";
  }
  ss << severity << ": ";
  ss << message;
  return ss.str();
}

std::string DiagnosticDetail::FormatWarning() const {
  return Format("warning");
}

std::string DiagnosticDetail::FormatError() const {
  return Format("error");
}

void DiagnosticPolicy::SetActionForWarnings(
    std::function<void(const DiagnosticDetail&)> functor) {
  on_warning_ = std::move(functor);
}

void DiagnosticPolicy::SetActionForErrors(
    std::function<void(const DiagnosticDetail&)> functor) {
  on_error_ = std::move(functor);
}

void DiagnosticPolicy::Warning(std::string message) const {
  DiagnosticDetail d;
  d.message = std::move(message);
  this->Warning(d);
}

void DiagnosticPolicy::Error(std::string message) const {
  DiagnosticDetail d;
  d.message = std::move(message);
  this->Error(d);
}

void DiagnosticPolicy::WarningDefaultAction(const DiagnosticDetail& detail) {
  log()->warn(detail.FormatWarning());
}

void DiagnosticPolicy::ErrorDefaultAction(const DiagnosticDetail& detail) {
  throw std::runtime_error(detail.FormatError());
}

void DiagnosticPolicy::Warning(const DiagnosticDetail& detail) const {
  if (on_warning_ == nullptr) {
    WarningDefaultAction(detail);
  } else {
    on_warning_(detail);
  }
}

void DiagnosticPolicy::Error(const DiagnosticDetail& detail) const {
  if (on_error_ == nullptr) {
    ErrorDefaultAction(detail);
  } else {
    on_error_(detail);
  }
}

}  // namespace internal
}  // namespace drake
