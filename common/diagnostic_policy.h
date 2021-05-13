#pragma once

#include <functional>
#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace internal {

/* Captures details about a warning or error condition.
Additional member fields might be added in future revisions. */
struct DiagnosticDetail {
  std::string message;

  // TODO(jwnimmer-tri) We probably should have some kind of to_string here
  // that collects all member fields' information.
};

/* A structured mechanism for functions that accept untrustworthy data to
report problems back to their caller.

By default, warnings will merely be logged into drake::log(), and errors will
throw exceptions.  The user can customize this behavior via the Set methods.

Note in particular that the user's replacement error callback is not required
to throw.  Code that calls policy.Error must be prepared to received control
back from that method; typically, this means "return;" should follow its use.
*/
class DiagnosticPolicy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiagnosticPolicy)
  DiagnosticPolicy() = default;

  /* Trigger a warning. */
  void Warning(std::string message) const;

  /* Trigger an error. */
  void Error(std::string message) const;

  /* (Advanced.) Trigger a warning. */
  void Warning(const DiagnosticDetail& detail) const;

  /* (Advanced.) Trigger an error. */
  void Error(const DiagnosticDetail& detail) const;

  /* Setting to nullptr restores the default behavior (i.e., to log). */
  void SetActionForWarnings(std::function<void(const DiagnosticDetail&)>);

  /* Setting to nullptr restores the default behavior (i.e., to throw). */
  void SetActionForErrors(std::function<void(const DiagnosticDetail&)>);

 private:
  std::function<void(const DiagnosticDetail&)> on_warning_;
  std::function<void(const DiagnosticDetail&)> on_error_;
};

}  // namespace internal
}  // namespace drake
