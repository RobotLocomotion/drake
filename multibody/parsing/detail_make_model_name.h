#pragma once

#include <optional>
#include <string>

#include "drake/multibody/parsing/detail_parsing_workspace.h"

namespace drake {
namespace multibody {
namespace internal {

// Calculates the model name of a new model instance, enforcing consistent
// naming across all parsers that use it.
//
// Rules:
// * Use parent_model_name as a scoped name prefix, if present.
// * Use auto renaming to avoid conflicts, if enabled.
//   * Subscripts start at 1 ("thing_1"), and are compact.
//   * Subscripts of different base names are independent.
std::string MakeModelName(std::string_view candidate_name,
                          const std::optional<std::string>& parent_model_name,
                          const ParsingWorkspace& workspace);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
