#pragma once

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
class System;

template <typename T>
class Diagram;

/** Provides a "Visitor Pattern" for System and Diagram.  We have many
algorithms that would like to recurse through a Diagram and its subsystems.  At
each step, we need to know if the current System is a Diagram (or not).  Rather than performing the dynamic_cast, or adding a virtual method to System, you may use the visitor pattern enabled by this class, e.g.:

@code
template <typename T>
class MySystemVisitor : public SystemVisitor {
  ...
}

MySystemVisitor<T> visitor;
system.accept(visitor);
@endcode

will call the correct `visit` overload.

@tparam_default_scalar
*/
template <typename T>
class SystemVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemVisitor)
  virtual ~SystemVisitor() = default;

  /** This method will be called by System<T>::accept() if the System *is not* a
  Diagram<T>. */
  virtual void visit(const System<T>& system) = 0;

  /** This method will be called by System<T>::accept() if the System *is* a
  Diagram<T>. */
  virtual void visit(const Diagram<T>& diagram) = 0;

 protected:
  SystemVisitor() = default;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemVisitor)
