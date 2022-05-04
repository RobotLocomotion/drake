#include "drake/multibody/parsing/detail_sdf_diagnostic.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

void CheckSupportedElements(
    const DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
    const std::set<std::string>& supported_elements) {
  CheckSupportedElements(diagnostic, root_element.get(), supported_elements);
}

void CheckSupportedElements(
    const DiagnosticPolicy& diagnostic,
    const sdf::Element* root_element,
    const std::set<std::string>& supported_elements) {

  sdf::ElementPtr element = root_element->GetFirstElement();
  while (element) {
    const std::string& element_name = element->GetName();
    if ((supported_elements.find(element_name) == supported_elements.end()) &&
        element->GetExplicitlySetInFile()) {
      internal::DiagnosticDetail detail;
      if (!element->FilePath().empty()) {
        detail.filename = element->FilePath();
      }
      detail.line = element->LineNumber();
      detail.message =
          std::string("Unsupported SDF element in ") +
          root_element->GetName() + std::string(": ") + element_name;
      // Unsupported elements in the drake namespace are errors.
      if (element_name.find("drake:") == 0) {
        diagnostic.Error(detail);
      } else {
        diagnostic.Warning(detail);
      }
    }
    element = element->GetNextElement();
  }
}

void CheckSupportedElementValue(
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr element,
    const std::string& expected) {
  sdf::ParamPtr value = element->GetValue();
  if (value->GetAsString() != expected) {
    internal::DiagnosticDetail detail;
    if (!element->FilePath().empty()) {
      detail.filename = element->FilePath();
    }
    detail.line = element->LineNumber();
    detail.message =
        std::string("Unsupported value for SDF element ") +
        element->GetName() + std::string(": ") + value->GetAsString();
    diagnostic.Warning(detail);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
