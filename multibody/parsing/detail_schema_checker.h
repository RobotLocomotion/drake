#pragma once

#include <filesystem>
#include <string>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

/* Here are three overloaded versions of CheckDocumentAgainstRncSchema(). They
all do roughly the same thing, with different inputs: validate a document
against a schema written in RelaxNG Compact format, emitting warnings and/or
errors to a diagnostic policy object. The return value is guaranteed to be zero
if the validation completes without error. Otherwise, if the value is positive:
it is an `errno` value (if file operations failed). If the value is negative,
it is a count of validation errors.
*/

/* Validate a document against a schema, both provided as files. */
int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document);

/* Validate a document, provided as a file, against a schema, whose contents
 * are provided as an in-memory string. */
int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::filesystem::path& document);

/* Validate a document, provided as an in-memory string, against a schema,
whose contents are provided as an in-memory string. The `document_filename`
is only used to construct diagnostic messages; the value should be chosen to
help the user interpret the messages. */
int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::string& document_contents,
    const std::string& document_filename);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

