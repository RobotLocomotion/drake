#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace systems {
namespace lcm {

/** A mapping from {bus_name: interface} with sugar for error checking with
nice error messages during access.

Note that this class is shallow-const.  A user of a const LcmBuses object
cannot add or remove buses, but can retrieve a non-const DrakeLcmInterface
pointer and then "modify" the object it points to by subscribing to a
channel. */
class LcmBuses final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LcmBuses)

  /** Constructs an empty mapping. */
  LcmBuses();

  ~LcmBuses();

  /** Returns the total number of buses. */
  int size() const;

  /** Finds the bus of the given name, or throws if there is no such bus.
  The return value is an alias into memory owned by the DiagramBuilder,
  and is never nullptr.

  @param description_of_caller is a noun phrase that will be used when creating
  an error message. A typical value would be something like "Camera 5", "Robot
  controller", "The default visualizer", or etc. */
  drake::lcm::DrakeLcmInterface* Find(
      std::string_view description_of_caller,
      const std::string& bus_name) const;

  /** Returns a list of all known bus_name keys. */
  std::vector<std::string_view> GetAllBusNames() const;

  /** Adds a bus. Throws if the bus is nullptr, or if there was already a bus
  of the same name. */
  void Add(std::string bus_name, drake::lcm::DrakeLcmInterface*);

 private:
  std::map<std::string, drake::lcm::DrakeLcmInterface*> buses_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
