#include "drake/manipulation/schunk_wsg/schunk_wsg_command_translator.h"
#include "drake/manipulation/schunk_wsg/gen/schunk_wsg_command.h"

#include "drake/lcmt_schunk_wsg_command.hpp"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

void SchunkWsgCommandTranslator::Deserialize(
    const void* lcm_message_bytes, int lcm_message_length,
    systems::VectorBase<double>* vector_base) const {
  auto command =
      dynamic_cast<SchunkWsgCommand<double>*>(vector_base);

  lcmt_schunk_wsg_command msg{};
  msg.decode(lcm_message_bytes, 0, lcm_message_length);

  command->set_utime(msg.utime);
  command->set_target_position_mm(msg.target_position_mm);
  command->set_force(msg.force);
}

void SchunkWsgCommandTranslator::Serialize(
    double, const systems::VectorBase<double>& vector_base,
    std::vector<uint8_t>* lcm_message_bytes) const {
  lcmt_schunk_wsg_command msg;
  const auto& command =
      dynamic_cast<const SchunkWsgCommand<double>&>(vector_base);
  msg.utime = static_cast<uint64_t>(command.utime());
  msg.target_position_mm = command.target_position_mm();
  msg.force = command.force();

  const int lcm_message_length = msg.getEncodedSize();
  lcm_message_bytes->resize(lcm_message_length);
  msg.encode(lcm_message_bytes->data(), 0, lcm_message_length);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
