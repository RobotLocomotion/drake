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
errors to a diagnostic policy object. The return value is true
if both schema parsing and validation complete without error.
*/

/* Validate a document against a schema, both provided as files. */
bool CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document);

/* Validate a document, provided as a file, against a schema, whose contents
are provided as an in-memory string. The `rnc_schema_filename` is used both to
resolve any `include` statements in the schema, and to construct diagnostic
messages.
*/
bool CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::filesystem::path& rnc_schema_filename,
    const std::filesystem::path& document);

/* Validate a document, provided as an in-memory string, against a schema,
whose contents are provided as an in-memory string. The `rnc_schema_filename`
is used both to resolve any `include` statements in the schema, and to
construct diagnostic messages. The `document_filename` is only used to
construct diagnostic messages; the value should be chosen to help the user
interpret the messages. */
bool CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::filesystem::path& rnc_schema_filename,
    const std::string& document_contents,
    const std::string& document_filename);

}  // namespace internal
}  // namespace multibody
}  // namespace drake

