#pragma once

namespace drake {
namespace assert {

/// Configures the DRAKE_ASSERT and DRAKE_DEMAND assertion failure handling
/// behavior.
///
/// By default, assertion failures will result in an ::abort().  If this method
/// has ever been called, failures will result in a thrown exception instead.
///
/// Assertion configuration has process-wide scope.  Changes here will affect
/// all assertions within the current process.
///
/// This method is intended for projects that consume Drake as a library,
/// including Drake's bindings for interpreted languages such as Python.  Code
/// within the Drake project itself should never invoke this method, nor depend
/// on the assertion failure behavior being set to any particular value.
void set_assertion_failure_to_throw_exception();

}  // namespace assert
}  // namespace drake
