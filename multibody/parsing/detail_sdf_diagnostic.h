#pragma once

#include <set>
#include <string>

#include <sdf/Element.hh>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_common.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper class to format diagnostic messages for a SDFormat data source.
class SDFormatDiagnostic {
 public:
  // Both @p diagnostic and @p data_source are aliased; their lifetime must
  // exceed that of this object.  @p file_extension is only used to indicate
  // the type of source when @p data_source is a literal string. It is copied
  // internally, so places no restrictions on the lifetime of the passed
  // parameter.
  // @pre diagnostic cannot be nullptr.
  // @pre data_source cannot be nullptr.
  SDFormatDiagnostic(const drake::internal::DiagnosticPolicy* diagnostic,
                     const DataSource* data_source,
                     const std::string& file_extension = "sdf");

  // Issues a warning for an ElementConstPtr.
  void Warning(const sdf::ElementConstPtr element,
               const std::string& message) const;

  // Issues an error for an ElementConstPtr.
  void Error(const sdf::ElementConstPtr element,
             const std::string& message) const;

  // Make a temporary policy that can be passed to a node-unaware parsing
  // function. The lifetime of this object, and the @p element should be
  // greater than the lifetime of the returned policy.
  drake::internal::DiagnosticPolicy MakePolicyForNode(
      const sdf::Element& element) const;

  // Warn about spec-documented elements ignored by Drake.
  void WarnUnsupportedElement(const sdf::ElementConstPtr element,
                              const std::string& tag) const;

  // Warn about spec-documented attributes ignored by Drake.
  void WarnUnsupportedAttribute(const sdf::ElementConstPtr element,
                                const std::string& attribute) const;

  // Copies all `errors` into this Diagnostic object.
  // Returns true if there were any errors.
  bool PropagateErrors(const sdf::Errors& errors) const;

 private:
  // Makes a diagnostic detail record based on an Element.
  drake::internal::DiagnosticDetail MakeDetail(
      const sdf::Element& element, const std::string& message) const;

  const drake::internal::DiagnosticPolicy* diagnostic_{};
  const DataSource* data_source_{};
  const std::string file_extension_;
};

// Checks that all child elements of @p root_element are in the set of @p
// supported_elements, and logs warnings/errors using @p diagnostic.
// Unsupported elements in the `drake:` namespace are errors, all others are
// warnings.  (see https://github.com/RobotLocomotion/drake/issues/16785 for
// some discussion of this rationale)
void CheckSupportedElements(const SDFormatDiagnostic& diagnostic,
                            sdf::ElementConstPtr root_element,
                            const std::set<std::string>& supported_elements);

void CheckSupportedElements(const SDFormatDiagnostic& diagnostic,
                            const sdf::Element* root_element,
                            const std::set<std::string>& supported_elements);

// Checks, for elements where there is only one supported value, that
// the element matches that value if it's present.
void CheckSupportedElementValue(const SDFormatDiagnostic& diagnostic,
                                sdf::ElementConstPtr root_element,
                                const std::string& element_name,
                                const std::string& expected);

// Move-appends all `input_errors` onto `output_errors`.
// Returns true if `input_errors` contains any error.
bool PropagateErrors(sdf::Errors&& input_errors, sdf::Errors* output_errors);

// Returns true iff the given report indicates an error, or false
// for warnings.
bool IsError(const sdf::Error& report);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
