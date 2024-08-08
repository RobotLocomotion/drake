#pragma once

#include <filesystem>
#include <string>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

enum class Strictness {
  kLax,     // Everything is a warning.
  kNormal,  // Use severity of underlying messages.
  kStrict,  // Everything is an error.
};

/* Here are several versions of CheckDocument*AgainstRncSchema*(). They
all do roughly the same thing, with different inputs: validate a document
against a schema written in RelaxNG Compact format, emitting warnings and/or
errors to a diagnostic policy object. The return value is true
if both schema parsing and validation complete without error.
*/

/* Validate a document against a schema, both provided as files. */
bool CheckDocumentFileAgainstRncSchemaFile(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document,
    Strictness strictness = Strictness::kNormal);

/* Validate a document, provided as an in-memory string, against a schema,
whose contents are provided as a file. The `document_filename` is only used to
construct diagnostic messages; the value should be chosen to help the user
interpret the messages. */
bool CheckDocumentStringAgainstRncSchemaFile(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::string& document_contents,
    const std::string& document_filename,
    Strictness strictness = Strictness::kNormal);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

