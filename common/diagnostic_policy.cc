#include "drake/common/diagnostic_policy.h"

#include <stdexcept>
#include <sstream>
#include <utility>

#include "drake/common/text_logging.h"

namespace drake {
namespace internal {

std::string DiagnosticDetail::Format(const std::string& severity) const {
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
  this->Warning(DiagnosticDetail({.message = std::move(message)}));
}

void DiagnosticPolicy::Error(std::string message) const {
  this->Error(DiagnosticDetail({.message = std::move(message)}));
}

void DiagnosticPolicy::WarningDefaultAction(
    const DiagnosticDetail& detail) {
  log()->warn(detail.FormatWarning());
}

void DiagnosticPolicy::ErrorDefaultAction(
    const DiagnosticDetail& detail) {
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
