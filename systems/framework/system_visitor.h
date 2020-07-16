#pragma once

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
class System;

template <typename T>
class Diagram;

/** Provides a "Visitor Pattern" for System and Diagram.  Rather than adding
more virtual methods to the System base class, or performing a dynamic_cast to
test if a System is a Diagram, you may use the visitor pattern enabled by this
class, e.g.:

@code
template <typename T>
class MySystemVisitor : public SystemVisitor {
  ...
}

MySystemVisitor<T> visitor;
system.Accept(visitor);
@endcode

will call the correct `Visit` overload.

@note This method does *not* recurse through the subsystems of a Diagram, but
that is easy to do: just call Diagram::GetSystems() in your visitor and then
call Accept on the subsystems.

@tparam_default_scalar
*/
template <typename T>
class SystemVisitor {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemVisitor)
  virtual ~SystemVisitor() = default;

  /** This method will be called by System<T>::accept() if the System *is not* a
  Diagram<T>. */
  virtual void VisitSystem(const System<T>& system) = 0;

  /** This method will be called by System<T>::accept() if the System *is* a
  Diagram<T>. */
  virtual void VisitDiagram(const Diagram<T>& diagram) = 0;

 protected:
  SystemVisitor() = default;
};

}  // namespace systems
}  // namespace drake
