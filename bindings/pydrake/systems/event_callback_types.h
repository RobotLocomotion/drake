#pragma once

#include <functional>
#include <optional>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event_status.h"

namespace drake {
namespace pydrake {

/* Because Python doesn't offer static type checking to help remind the user to
return an EventStatus from an event handler function, we'll bind the callback
as optional<> to allow the user to omit a return statement. (When declaring a
periodic event in C++, the user-provided callback function is similarly
overloaded to return either EventStatus or void. We can't overload based on
return values in pybind11, so that's another reason we'll use optional<> in
Python for the same effect.) */
template <typename... Args>
using EventSignature = std::optional<systems::EventStatus>(Args...);

/* Declare the handler signature within std::function. */
template <typename... Args>
using EventCallback = std::function<EventSignature<Args...>>;

/* Declare the handler signature as a python function object with compile-time
readable type signature. */
#ifdef PYDRAKE_USE_PYBIND11
template <typename... Args>
using PyEventCallback = py::typing::Callable<EventSignature<Args...>>;
#else   // PYDRAKE_USE_NANOBIND
template <typename... Args>
using PyEventCallback = py::typed<py::callable, EventSignature<Args...>>;
#endif  // PYDRAKE_USE_PYBIND11

/* Declare the various callback types that will appear as parameters of
bindings, such that automatic documentation has correct type signatures. */
template <typename T>
using PyPublishCallback = PyEventCallback<const systems::Context<T>*>;
template <typename T>
using PyDiscreteUpdateCallback =
    PyEventCallback<const systems::Context<T>*, systems::DiscreteValues<T>*>;
template <typename T>
using PyUnrestrictedUpdateCallback =
    PyEventCallback<const systems::Context<T>*, systems::State<T>*>;

}  // namespace pydrake
}  // namespace drake
