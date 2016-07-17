#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/system3.h"
#include "drake/systems/framework/primitives/diagram3.h"

namespace drake {
namespace systems {

/** A %Cascade is a concrete SystemDiagram containing exactly two compatible
subsystems with the output of the first serving as input to the second. The
input of the first subsystem is also the input of the %Cascade, and the output
of the second subsystem is also the output of the %Cascade.

@tparam T The type of numerical values in this system diagram. Must be a
          valid Eigen scalar. **/
template <typename T>
class Cascade3 : public Diagram3<T> {
 public:
  /** Takes over ownership of the two System objects and connects the outputs
  of `first` to the inputs of `second`. The number and types of output
  and input ports must match. The input of `first` becomes the overall
  input of the %Cascade, and the output of `second` becomes the overall
  output of %Cascade. Neither system may be empty.
  @throws std::logic_error One or both System arguments was empty.
  @throws std::logic_error Number of output and input ports didn't match. **/
  Cascade3(const std::string& name, std::unique_ptr<System3<T>> first,
           std::unique_ptr<System3<T>> second)
      : Diagram3(name) {
    if (!first || !second)
      throw std::logic_error("Cascade: empty Systems not allowed.");

    const int out1 = first->get_num_output_ports();
    const int in2 = second->get_num_input_ports();

    if (in2 != out1) {
      throw std::logic_error(
          "Cascade3: first System has "
          + std::to_string(out1) + " output ports but second System has "
          + std::to_string(in2) + " input ports. They must match.");
    }

    auto first_system = AddSubsystem(std::move(first));
    auto second_system = AddSubsystem(std::move(second));

    // Input port numbers for Cascade match first system's.
    for (int port = 0; port < first_system->get_num_input_ports(); ++port) {
      const int port_num = InheritInputPort(first_system, port);
      DRAKE_ABORT_UNLESS(port_num == port);
    }

    // Connect up the interior ports.
    for (int port = 0; port < out1; ++port)
      Connect(first_system, port, second_system, port);

    // Output port numbers for Cascade match second system's.
    for (int port = 0; port < second_system->get_num_output_ports(); ++port) {
      const int port_num = InheritOutputPort(second_system, port);
      DRAKE_ABORT_UNLESS(port_num == port);
    }
  }
};

}  // namespace systems
}  // namespace drake
