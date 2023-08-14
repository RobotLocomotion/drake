#include "drake/systems/framework/event_status.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

void EventStatus::ThrowOnFailure(const char* function_name) const {
  if (!failed()) return;
  DRAKE_THROW_UNLESS(function_name != nullptr);

  /* Attempt to identify the relevant subsystem in human-readable terms. If no
  subsystem was provided we just call it "System". Otherwise, we obtain its
  type and its name and call it "type system 'name'", e.g.
  "MultibodyPlant<double> system 'my_plant'". */
  const std::string system_id =
      system() == nullptr
          ? "System"
          : fmt::format("{} system '{}'", system()->GetSystemType(),
                        system()->GetSystemPathname());

  throw std::runtime_error(
      fmt::format("{}(): An event handler in {} failed with message: \"{}\".",
                  function_name, system_id, message()));
}

}  // namespace systems
}  // namespace drake
