#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace systems {
namespace lcm {

SerializerInterface::~SerializerInterface() {}

std::shared_ptr<AbstractValue> SerializerInterface::CreateDefaultValueShared()
    const {
  return CreateDefaultValue();
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
