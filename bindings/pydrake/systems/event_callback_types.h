#pragma once

#include <functional>
#include <optional>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/system.h"

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
using CppEventSignature = std::optional<systems::EventStatus>(Args...);

/* Declare the handler signature within std::function. */
template <typename... Args>
using CppEventCallback = std::function<CppEventSignature<Args...>>;

/* Declare the handler signature as a python function object with compile-time
readable type signature. */
#ifdef PYDRAKE_USE_PYBIND11
template <typename... Args>
using PyEventCallback = py::typing::Callable<CppEventSignature<Args...>>;
#else   // PYDRAKE_USE_NANOBIND
template <typename... Args>
using PyEventCallback = py::typed<py::callable, CppEventSignature<Args...>>;
#endif  // PYDRAKE_USE_PYBIND11

/* Declare the various callback types that will appear as parameters of
bindings, such that automatic documentation has correct type signatures. Each
PyFooCallback is paired with a CppFooCallback that has the identical argument
list, so that bindings can always py::cast from PyFooCallback<T> to
CppFooCallback<T> without needing to cross-check argument alignment. */

// For LeafSystem::Declare{Initialization,Periodic,PerStep,Forced}*Event.
template <typename T>
using PyPublishCallback = PyEventCallback<const systems::Context<T>*>;
template <typename T>
using CppPublishCallback = CppEventCallback<const systems::Context<T>*>;

template <typename T>
using PyDiscreteUpdateCallback =
    PyEventCallback<const systems::Context<T>*, systems::DiscreteValues<T>*>;
template <typename T>
using CppDiscreteUpdateCallback =
    CppEventCallback<const systems::Context<T>*, systems::DiscreteValues<T>*>;

template <typename T>
using PyUnrestrictedUpdateCallback =
    PyEventCallback<const systems::Context<T>*, systems::State<T>*>;
template <typename T>
using CppUnrestrictedUpdateCallback =
    CppEventCallback<const systems::Context<T>*, systems::State<T>*>;

// For the PublishEvent and UnrestrictedUpdateEvent constructors.
template <typename T>
using PyPublishEventCallback = PyEventCallback<const systems::Context<T>*,
    const systems::PublishEvent<T>*>;
template <typename T>
using CppPublishEventCallback = CppEventCallback<const systems::Context<T>*,
    const systems::PublishEvent<T>*>;

template <typename T>
using PyPublishEventSystemCallback = PyEventCallback<const systems::System<T>*,
    const systems::Context<T>*, const systems::PublishEvent<T>*>;
template <typename T>
using CppPublishEventSystemCallback =
    CppEventCallback<const systems::System<T>*, const systems::Context<T>*,
        const systems::PublishEvent<T>*>;

template <typename T>
using PyUnrestrictedUpdateEventCallback =
    PyEventCallback<const systems::Context<T>*,
        const systems::UnrestrictedUpdateEvent<T>*, systems::State<T>*>;
template <typename T>
using CppUnrestrictedUpdateEventCallback =
    CppEventCallback<const systems::Context<T>*,
        const systems::UnrestrictedUpdateEvent<T>*, systems::State<T>*>;

template <typename T>
using PyUnrestrictedUpdateEventSystemCallback =
    PyEventCallback<const systems::System<T>*, const systems::Context<T>*,
        const systems::UnrestrictedUpdateEvent<T>*, systems::State<T>*>;
template <typename T>
using CppUnrestrictedUpdateEventSystemCallback =
    CppEventCallback<const systems::System<T>*, const systems::Context<T>*,
        const systems::UnrestrictedUpdateEvent<T>*, systems::State<T>*>;

}  // namespace pydrake
}  // namespace drake
