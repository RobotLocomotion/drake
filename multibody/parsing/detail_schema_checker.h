#pragma once

#include <filesystem>
#include <string>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::filesystem::path& rnc_schema,
    const std::filesystem::path& document);

int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::filesystem::path& document);

int CheckDocumentAgainstRncSchema(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const std::string& rnc_schema_string,
    const std::string& document,
    const std::filesystem::path& document_filename);



}  // namespace internal
}  // namespace multibody
}  // namespace drake

