#pragma once

#include <functional>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace internal {

/* Captures details about a warning or error condition.
Additional member fields might be added in future revisions. */
struct DiagnosticDetail {
  std::optional<std::string> filename;
  std::optional<int> line;
  std::string message;

  /* @return a formatted diagnostic message like one of:

         file.txt:35: severity: some message
         file.txt: severity: some message
         severity: some message

      depending on which fields are populated.

     @pre severity must not be empty.
   */
  std::string Format(const std::string& severity) const;

  /* @return the result of Format("warning").  */
  std::string FormatWarning() const;

  /* @return the result of Format("error").  */
  std::string FormatError() const;
};

/* A structured mechanism for functions that accept untrustworthy data to
report problems back to their caller.

By default, warnings will merely be logged into drake::log(), and errors will
throw exceptions.  The user can customize this behavior via the Set methods.

Note in particular that the user's replacement error callback is not required
to throw.  Code that calls policy.Error must be prepared to receive control
back from that method; typically, this means "return;" should follow its use.
*/
class DiagnosticPolicy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiagnosticPolicy);
  DiagnosticPolicy() = default;

  /* Triggers a warning. */
  void Warning(std::string message) const;

  /* Triggers an error. */
  void Error(std::string message) const;

  /* (Advanced) Triggers a warning. */
  void Warning(const DiagnosticDetail& detail) const;

  /* (Advanced) Triggers an error. */
  void Error(const DiagnosticDetail& detail) const;

  /* (Advanced) Processes a warning using default behavior. */
  static void WarningDefaultAction(const DiagnosticDetail& detail);

  /* (Advanced) Processes an error using default behavior. */
  [[noreturn]] static void ErrorDefaultAction(const DiagnosticDetail& detail);

  /* Replaces the behavior for warnings with the supplied functor.
     Setting to nullptr restores the default behavior (i.e., to log). */
  void SetActionForWarnings(std::function<void(const DiagnosticDetail&)>);

  /* Replaces the behavior for errors with the supplied functor.
     Setting to nullptr restores the default behavior (i.e., to throw). */
  void SetActionForErrors(std::function<void(const DiagnosticDetail&)>);

 private:
  std::function<void(const DiagnosticDetail&)> on_warning_;
  std::function<void(const DiagnosticDetail&)> on_error_;
};

}  // namespace internal
}  // namespace drake
