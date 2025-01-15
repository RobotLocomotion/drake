#pragma once

#include <functional>
#include <memory>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context_base.h"

namespace drake {
namespace pydrake {

// The C++ framework uses ValueProducer::AllocateCallback to allocate storage
// for values in the Context, and that function returns a unique_ptr. Python
// cannot return unique_ptr, so we need to wrap the call to match signatures.
// The argument is the Python callback; the return value is the C++ callback.
std::function<std::unique_ptr<AbstractValue>()>
MakeCppCompatibleAllocateCallback(py::function allocate);

// For parity with `allocate`, we also provide a wrapper for `calc`. It doesn't
// do any special tricks (our nominal WrapCallbacks() technique could've handled
// it), but it's easier to avoid WrapCallbacks entirely once we need to special-
// case the `allocate` callback.
std::function<void(const systems::ContextBase&, AbstractValue*)>
MakeCppCompatibleCalcCallback(std::function<void(py::object, py::object)> calc);

}  // namespace pydrake
}  // namespace drake
