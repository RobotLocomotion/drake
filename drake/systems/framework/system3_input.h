#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/system3_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/** An %InputPort represents an external data input to a System. This or another
System may provide an OutputPort which provides values that are passed into an
%InputPort.

In a system diagram, output ports of some contained subsystems will be connected
to input ports of others. When the connection is made, the connected OutputPort
is recorded here. Later that will be used to construct dependency information
in the Context so that a change of value of an output port will properly
invalidate computations that are dependent on a connected input port. Note
that an %InputPort is connected to at most one OutputPort, but an OutputPort
may be the source for many inputs.

An %InputPort contains a specification of what properties an OutputPort must
have in order to be an acceptable mate for that %InputPort. By default any
OutputPort is acceptable, and the value of the %InputPort will be the
AbstractValue of the OutputPort. The derived class `VectorInputPort<T>` will
only accept a `VectorOutputPort<T>` of a specified length or of any length.

In addition to type, and %InputPort may set restrictions on the sample rates
that are acceptable. For example, it may specify that only continuous outputs
are allowed, or only discrete. **/

// TODO(sherm1) Deal with sample rates.
class InputPort3 {
 public:
  /** Create an %InputPort that will accept any OutputPort as a connection. **/
  InputPort3() {}

  /** Connect this %InputPort to a compatible OutputPort. If the OutputPort
  is null, this disconnects the %InputPort from whatever it may have been
  connected to before. The output port must be owned by a system at the same
  level or higher (closer to root) in the system diagram as the system that owns
  this %InputPort. The higher case occurs when the %InputPort has been
  inherited by a parent or ancestor of the system that owns it.

  @throws std::logic_error The OutputPort was not an acceptable connection
                           for this %InputPort. **/

  // TODO(sherm1) Check that the output port's subsystem level makes sense.
  void ConnectTo(const OutputPort3* output_port) {
    CheckOutputPort(output_port);
    output_port_ = output_port;
  }

  /** Given an OutputPort proposed as a connection for this %InputPort,
  throw an error message if the OutputPort is not acceptable. A null OutputPort
  is allowed and is interpreted as "disconnect". **/
  void CheckOutputPort(const OutputPort3* proposed) const {
    if (proposed != nullptr) DoCheckOutputPort(*proposed);
  }

  /** If connected, return a pointer to the output port otherwise `nullptr`. **/
  const OutputPort3* get_connection() const { return output_port_; }

 protected:
  /** A concrete %InputPort can override this method to restrict the kinds
  of OutputPort objects that are acceptable. If you find the proposed connection
  unacceptable, throw an std::logic_error with a message explaining the
  problem. **/
  virtual void DoCheckOutputPort(const OutputPort3& proposed) const {}

 private:
  friend class AbstractSystem3;  // Allow call to set_owner().

  // Set backpointer; no ownership implied.
  void set_owner(AbstractSystem3* owner, int input_port_num) {
    system_ = owner;
    input_port_num_ = input_port_num;
  }

  // This AbstractSystem is the owner of this InputPort, where it has this port
  // number.
  class AbstractSystem3* system_{};
  int input_port_num_{-1};

  // This is nullptr if this InputPort is not connected to an OutputPort.
  const OutputPort3* output_port_{};
};

/** A %VectorInputPort extends InputPort with the restriction that only
VectorOutputPort connections are acceptable, and the value will be of type
VectorInterface. You may optionally require that only a specified length of
vector is acceptable; otherwise any length is allowed. **/
template <typename T>
class VectorInputPort3 : public InputPort3 {
 public:
  /** Create a %VectorInputPort that accepts any length VectorOutputPort as
  a connection. **/
  VectorInputPort3() {}

  /** Create a %VectorInputPort3 that only permits a connection to a particular
  length of VectorOutputPort3. **/
  explicit VectorInputPort3(int required_length)
      : required_length_(required_length) {
    DRAKE_ABORT_UNLESS(required_length >= -1);
    // TODO(sherm1) Should be an std::logic_error rather than abort().
  }

 private:
  void DoCheckOutputPort(const OutputPort3& proposed) const override {
    using namespace std::literals::string_literals;

    auto vector_output = dynamic_cast<const VectorOutputPort3<T>*>(&proposed);
    if (vector_output == nullptr) {
      throw std::logic_error(
          "VectorInputPort3::ConnectTo(): Proposed connection was not a "s +
          NiceTypeName::Get<VectorOutputPort3<T>>() + ".");
    }
    const int output_size = vector_output->get_model_vector().size();
    if (required_length_ >= 0 && output_size != required_length_) {
      throw std::out_of_range(
          "VectorInputPort3::ConnectTo(): "s +
          "Proposed connection was a VectorOutputPort3 of size " +
          std::to_string(output_size) + " but must be size " +
          std::to_string(required_length_) + " for this VectorInputPort3.");
    }
    // Looks good if we get here.
  }

  int required_length_{-1};  // Means "any length is OK".
};

}  // namespace systems
}  // namespace drake
