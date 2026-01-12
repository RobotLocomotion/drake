#include "drake/systems/lcm/lcm_buses.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

using drake::lcm::DrakeLcmInterface;

namespace drake {
namespace systems {
namespace lcm {

LcmBuses::LcmBuses() = default;

LcmBuses::~LcmBuses() = default;

int LcmBuses::size() const {
  return buses_.size();
}

DrakeLcmInterface* LcmBuses::Find(std::string_view description_of_caller,
                                  const std::string& bus_name) const {
  auto iter = buses_.find(bus_name);
  if (iter != buses_.end()) {
    DrakeLcmInterface* result = iter->second;
    DRAKE_DEMAND(result != nullptr);
    return result;
  }
  throw std::runtime_error(
      fmt::format("{} requested an LCM bus '{}' that does not exist",
                  description_of_caller, bus_name));
}

std::vector<std::string_view> LcmBuses::GetAllBusNames() const {
  std::vector<std::string_view> result;
  result.reserve(buses_.size());
  for (const auto& key_value : buses_) {
    result.push_back(key_value.first);
  }
  return result;
}

void LcmBuses::Add(std::string bus_name, DrakeLcmInterface* bus) {
  DRAKE_THROW_UNLESS(bus != nullptr);
  const bool inserted = buses_.emplace(std::move(bus_name), bus).second;
  if (!inserted) {
    throw std::runtime_error(fmt::format(
        "An LCM bus with name '{}' has already been defined", bus_name));
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
