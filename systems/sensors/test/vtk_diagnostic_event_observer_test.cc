#include "drake/systems/sensors/vtk_diagnostic_event_observer.h"

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkNew.h>  // vtkCommonCore

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

GTEST_TEST(VtkDiagnosticEventObserverTest, General) {
  // Prepare a DiagnosticPolicy that simply accumulates its messages.
  std::vector<std::string> messages;
  DiagnosticPolicy diagnostic_policy;
  diagnostic_policy.SetActionForWarnings([&](const DiagnosticDetail& detail) {
    messages.push_back(detail.FormatWarning());
  });
  diagnostic_policy.SetActionForErrors([&](const DiagnosticDetail& detail) {
    messages.push_back(detail.FormatError());
  });

  // Create the device under test.
  vtkNew<VtkDiagnosticEventObserver> dut;
  dut->set_diagnostic(&diagnostic_policy);

  // Create a dummy object that will generate messages observed by the dut.
  vtkNew<vtkObject> dummy;
  dummy->AddObserver(vtkCommand::ErrorEvent, dut);
  dummy->AddObserver(vtkCommand::WarningEvent, dut);

  // Generate messages.
  const char* const error_message =
      "text for error\n"
      "next line\n\n";
  const char* const warning_message =
      "text for warning\n"
      "next line\n\n";
  dummy->InvokeEvent(
      vtkCommand::ErrorEvent,
      const_cast<void*>(static_cast<const void*>(error_message)));
  dummy->InvokeEvent(
      vtkCommand::WarningEvent,
      const_cast<void*>(static_cast<const void*>(warning_message)));

  // Check for what came out.
  EXPECT_THAT(messages,
              testing::ElementsAre("error: text for error: next line",
                                   "warning: text for warning: next line"));
}

}  // namespace
}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
