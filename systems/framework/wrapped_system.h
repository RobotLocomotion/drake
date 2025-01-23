#pragma once

#include <memory>

#include "drake/systems/framework/diagram.h"

namespace drake {
namespace systems {
namespace internal {

/* WrappedSystem<T> implements the System<T> interface using a pre-existing
System<T> object held internally as a shared_ptr (i.e., the "decorator" design
pattern). All calls are forwarded to the nested system.

Currently, this is only used to facilitate the scalar conversion for leaf
systems implemented in Python.

For the implementation, the easiest way to forward all System calls along to
another System is actually through the composite pattern (aka part-whole) that
we've already implemented as Diagram. The WrappedSystem is-a Diagram that has
exactly one subsystem.

@tparam_default_scalar */
template <typename T>
class WrappedSystem final : public Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WrappedSystem);

  /* Wraps the given system, which must not be nullptr. Even though ownership is
  shared, the given `system` must NOT be mutated in any way (e.g., added to a
  different Diagram).
  @tparam Ignored is ignored; it's only job is to allow us to define this
  constructor in wrapped_system_builder.cc instead of wrapped_system.cc. */
  template <typename Ignored = void>
  explicit WrappedSystem(std::shared_ptr<System<T>> system);

  /* Scalar-converting copy constructor. See @ref system_scalar_conversion.
  Ideally this would not be inline, but with our weird linking situation it's
  vastly simpler to keep it here. */
  template <typename U>
  explicit WrappedSystem(const WrappedSystem<U>& other)
      : Diagram<T>(systems::SystemTypeTag<WrappedSystem>{}, other) {}

  ~WrappedSystem() final;

  /* Returns the underlying System. */
  const System<T>& unwrap() const;
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
