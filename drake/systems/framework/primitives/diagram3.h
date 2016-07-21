#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/system3.h"

namespace drake {
namespace systems {

/** A %Diagram is a concrete System that contains other System objects as
subsystems and wires them together, but adds no new content. The %Diagram may
have input and output ports but they must be inherited from the contained 
subsystems.

Inherited outputs are evaluated by forwarding to the subystem that owns the
original output port. Derivatives and updates are evaluated by invoking the
corresponding methods on the contained subsystems and aggregating the results.

@tparam T The type of numerical values in this %SystemDiagram. Must be a
          valid Eigen scalar. **/
template <typename T>
class Diagram3: public System3<T> {
public:
  /** Default constructor creates a diagram with no subsystems and no input
  or output ports. Use `AddSubsystem()`, `InheritInputPort()`, 
  `InheritOutputPort()` and `Connect()` methods to fill in this diagram. **/
  explicit Diagram3(const std::string& name) : System3<T>(name) {}
private:
  // Implement System3<T> interface.

  std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const final {
    return std::make_unique<Context3<double>>();
  }

  // Don't need any additional resources.

  // Calculation consists of finding the subsystem that owns the actual
  // output port and making it do the evaluation.
  void DoCalcOutputPort(const AbstractContext3& context, int port_num,
                        AbstractValue* value) const final {
    throw std::logic_error(
        "Diagram3::DoCalcOutputPort(): a diagram shouldn't own any ports.");
  }
};

}  // namespace systems
}  // namespace drake
