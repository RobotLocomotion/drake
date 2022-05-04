#pragma once

#include <set>
#include <string>

#include <sdf/Element.hh>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

/// Checks that all child elements of @p root_element are in the set of @p
/// supported_elements, and logs warnings/errors using @p diagnostic.
void CheckSupportedElements(
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
    const std::set<std::string>& supported_elements);

void CheckSupportedElements(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const sdf::Element* root_element,
    const std::set<std::string>& supported_elements);

/// Checks that, for elements where there  is only one supported  value, that
/// the element matches that value.
void CheckSupportedElementValue(
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr element,
    const std::string& expected);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
