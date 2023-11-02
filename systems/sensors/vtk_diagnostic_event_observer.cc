#include "drake/systems/sensors/vtk_diagnostic_event_observer.h"

#include <string>

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

VtkDiagnosticEventObserver::VtkDiagnosticEventObserver() = default;

VtkDiagnosticEventObserver::~VtkDiagnosticEventObserver() = default;

namespace {
/* Adjusts the given VTK error or warning message to better conform to Drake
style. (VTK likes to spew newlines, but that's annoying in a timestamped and
record-oriented console log file.) */
std::string ConvertVtkMessageToDrakeStyle(const char* message) {
  DRAKE_DEMAND(message != nullptr);
  std::string result(message);
  // Remove trailing newlines.
  while (result.size() > 0 && result.back() == '\n') {
    result.resize(result.size() - 1);
  }
  // Replace embedded newlines.
  while (true) {
    auto pos = result.find('\n');
    if (pos == std::string::npos) {
      break;
    }
    result = result.replace(pos, 1, ": ");
  }
  return result;
}
}  // namespace

// NOLINTNEXTLINE(runtime/int) To match the VTK signature.
void VtkDiagnosticEventObserver::Execute(vtkObject*, unsigned long event,
                                         void* calldata) {
  switch (event) {
    case vtkCommand::ErrorEvent: {
      DRAKE_DEMAND(diagnostic_ != nullptr);
      const char* message = static_cast<char*>(calldata);
      diagnostic_->Error(ConvertVtkMessageToDrakeStyle(message));
      return;
    }
    case vtkCommand::WarningEvent: {
      DRAKE_DEMAND(diagnostic_ != nullptr);
      const char* message = static_cast<char*>(calldata);
      diagnostic_->Warning(ConvertVtkMessageToDrakeStyle(message));
      return;
    }
  }
}

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
