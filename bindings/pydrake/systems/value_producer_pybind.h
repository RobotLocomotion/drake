#pragma once

#include <functional>
#include <memory>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context_base.h"

namespace drake {
namespace pydrake {

// The `calc` callback of an abstract output port or value producer, which
// receives a context and an output AbstractValue.
#ifdef PYDRAKE_USE_PYBIND11
using PyCalcAbstractCallback =
    py::typing::Callable<void(py::object, py::object)>;
#else   // PYDRAKE_USE_NANOBIND
using PyCalcAbstractCallback =
    py::typed<py::callable, void(py::object, py::object)>;
#endif  // PYDRAKE_USE_PYBIND11

// The C++ framework uses ValueProducer::AllocateCallback to allocate storage
// for values in the Context, and that function returns a unique_ptr. Python
// cannot return unique_ptr, so we need to wrap the call to match signatures.
// The argument is the Python callback; the return value is the C++ callback.
std::function<std::unique_ptr<AbstractValue>()>
MakeCppCompatibleAllocateCallback(py::callable allocate);

// For parity with `allocate`, we also provide a wrapper for `calc`. It doesn't
// do any special tricks (our nominal WrapCallbacks() technique could've handled
// it), but it's easier to avoid WrapCallbacks entirely once we need to special-
// case the `allocate` callback.
std::function<void(const systems::ContextBase&, AbstractValue*)>
MakeCppCompatibleCalcCallback(std::function<void(py::object, py::object)> calc);

// The "borrowed" variants below are like the functions above, except that the
// returned C++ callback holds only a non-owning pointer to the Python callable.
// Holding a strong reference inside a std::function is invisible to Python's
// garbage collection, so a callable that captures its owner would leak the
// owner. Callers must guarantee the callable's lifetime elsewhere, e.g., by
// using internal::ensure_callback_lifetime in the binding.
std::function<std::unique_ptr<AbstractValue>()> MakeBorrowedAllocateCallback(
    PyObject* allocate_ptr);

std::function<void(const systems::ContextBase&, AbstractValue*)>
MakeBorrowedCalcCallback(PyObject* calc_ptr);

}  // namespace pydrake
}  // namespace drake
