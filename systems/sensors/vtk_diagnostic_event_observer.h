#pragma once

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCommand.h>  // vtkCommonCore
#include <vtkObject.h>   // vtkCommonCore

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

/* Forwards warnings and/or errors from VTK into a Drake DiagnosticPolicy. */
class DRAKE_NO_EXPORT VtkDiagnosticEventObserver final : public vtkCommand {
 public:
  VtkDiagnosticEventObserver();
  ~VtkDiagnosticEventObserver() final;

  static VtkDiagnosticEventObserver* New() {
    return new VtkDiagnosticEventObserver;
  }

  /* Sets where ErrorEvent and WarningEvent should forward to. This must be
  called immediately after construction. */
  void set_diagnostic(const drake::internal::DiagnosticPolicy* diagnostic) {
    DRAKE_DEMAND(diagnostic != nullptr);
    diagnostic_ = diagnostic;
  }

  // NOLINTNEXTLINE(runtime/int) To match the VTK signature.
  void Execute(vtkObject*, unsigned long event, void* calldata) final;

 private:
  const drake::internal::DiagnosticPolicy* diagnostic_{};
};

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
