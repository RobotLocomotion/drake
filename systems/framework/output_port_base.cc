#include "drake/systems/framework/output_port_base.h"

#include <utility>

namespace drake {
namespace systems {

OutputPortBase::OutputPortBase(
    internal::SystemMessageInterface* owning_system, std::string name,
    OutputPortIndex index, DependencyTicket ticket, PortDataType data_type,
    int size)
    : PortBase("Output", owning_system, std::move(name), index, ticket,
               data_type, size) {}

OutputPortBase::~OutputPortBase() = default;

}  // namespace systems
}  // namespace drake
