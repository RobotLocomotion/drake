#pragma once

#include <set>
#include <string>

#include <drake_vendor/sdf/Element.hh>

#include "drake/common/diagnostic_policy.h"

namespace drake {
namespace multibody {
namespace internal {

/// Checks that all child elements of @p root_element are in the set of @p
/// supported_elements, and logs warnings/errors using @p diagnostic.
/// Unsupported elements in the `drake:` namespace are errors, all others are
/// warnings.  (see https://github.com/RobotLocomotion/drake/issues/16785 for
/// some discussion of this rationale)
void CheckSupportedElements(
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
    const std::set<std::string>& supported_elements);

void CheckSupportedElements(
    const drake::internal::DiagnosticPolicy& diagnostic,
    const sdf::Element* root_element,
    const std::set<std::string>& supported_elements);

/// Checks, for elements where there is only one supported value, that
/// the element matches that value if it's present.
void CheckSupportedElementValue(
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
    const std::string& element_name,
    const std::string& expected);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
