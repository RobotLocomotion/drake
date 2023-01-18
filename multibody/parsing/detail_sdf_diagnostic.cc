#include "drake/multibody/parsing/detail_sdf_diagnostic.h"

#include "drake/common/drake_assert.h"

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
  DRAKE_DEMAND(root_element != nullptr);

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
      // Unsupported elements in the drake namespace are errors.
      if (element_name.find("drake:") == 0) {
        detail.message =
            std::string("Unsupported SDFormat element in ") +
            root_element->GetName() + std::string(": ") + element_name;
        diagnostic.Error(detail);
      } else {
        detail.message =
            std::string("Ignoring unsupported SDFormat element in ") +
            root_element->GetName() + std::string(": ") + element_name;
        diagnostic.Warning(detail);
      }
    }
    element = element->GetNextElement();
  }
}

void CheckSupportedElementValue(
    const drake::internal::DiagnosticPolicy& diagnostic,
    sdf::ElementPtr root_element,
    const std::string& element_name,
    const std::string& expected) {
  DRAKE_DEMAND(root_element != nullptr);

  if (!root_element->HasElement(element_name)) {
    return;
  }

  sdf::ElementPtr element = root_element->GetElement(element_name);
  if (!element->GetExplicitlySetInFile()) {
    return;
  }

  sdf::ParamPtr value = element->GetValue();
  if (value->GetAsString() != expected) {
    internal::DiagnosticDetail detail;
    if (!element->FilePath().empty()) {
      detail.filename = element->FilePath();
    }
    detail.line = element->LineNumber();
    detail.message =
        std::string("Unsupported value for SDFormat element ") +
        element->GetName() + std::string(": ") + value->GetAsString();
    diagnostic.Warning(detail);
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
