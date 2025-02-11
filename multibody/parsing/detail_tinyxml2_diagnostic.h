#pragma once

#include <string>

#include <tinyxml2.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_common.h"

namespace drake {
namespace multibody {
namespace internal {

// Helper class to format diagnostic messages for a TinyXML2 data source.
class TinyXml2Diagnostic {
 public:
  // Both @p diagnostic and @p data_source are aliased; their lifetime must
  // exceed that of this object.  @p file_extension is only used for formatting
  // diagnostics from file_contents sources. It is copied internally, so places
  // no restrictions on the lifetime of the passed parameter.
  // @pre diagnostic cannot be nullptr.
  // @pre data_source cannot be nullptr.
  TinyXml2Diagnostic(const drake::internal::DiagnosticPolicy* diagnostic,
                     const DataSource* data_source,
                     const std::string& file_extension = "urdf");

  // Issues a warning for an XMLNode.
  void Warning(const tinyxml2::XMLNode& location,
               const std::string& message) const;

  // Issues an error for an XMLNode.
  void Error(const tinyxml2::XMLNode& location,
             const std::string& message) const;

  // Make a temporary policy that can be passed to a node-unaware parsing
  // function. The lifetime of this object, and the @p location should be
  // greater than the lifetime of the returned policy.
  drake::internal::DiagnosticPolicy MakePolicyForNode(
      const tinyxml2::XMLNode* location) const;

  // Warn about spec-documented elements ignored by Drake.
  void WarnUnsupportedElement(const tinyxml2::XMLElement& node,
                              const std::string& tag) const;

  // Warn about spec-documented attributes ignored by Drake.
  void WarnUnsupportedAttribute(const tinyxml2::XMLElement& node,
                                const std::string& attribute) const;

 private:
  // Makes a diagnostic detail record based on an XMLNode.
  drake::internal::DiagnosticDetail MakeDetail(
      const tinyxml2::XMLNode& location, const std::string& message) const;

  const drake::internal::DiagnosticPolicy* diagnostic_{};
  const DataSource* data_source_{};
  const std::string file_extension_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
