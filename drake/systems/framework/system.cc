#include "drake/systems/framework/system.h"

#include <atomic>
#include <iomanip>
#include <ios>
#include <regex>

#include "drake/common/default_scalars.h"
#include "drake/common/never_destroyed.h"

using std::setfill;
using std::setw;
using std::hex;

namespace drake {
namespace systems {

std::string SystemImpl::GetMemoryObjectName(
    const std::string& nice_type_name, int64_t address) {

  // Remove the template parameter(s).
  const std::string type_name_without_templates = std::regex_replace(
      nice_type_name, std::regex("<.*>$"), std::string());

  // Replace "::" with "/" because ":" is the System::GetPath separator.
  const std::string default_name = std::regex_replace(
      type_name_without_templates, std::regex(":+"), std::string("/"));

  // Append the address spelled like "@0123456789abcdef".
  std::ostringstream result;
  result << default_name << '@' << setfill('0') << setw(16) << hex << address;
  return result.str();
}

namespace {
// Return an ever-increasing sequence number
uint64_t get_next_id() {
  static never_destroyed<std::atomic<uint64_t>> next_id{0};
  return next_id.access()++;
}
}  // namespace

std::string SystemImpl::MakeRandomName() {
  std::ostringstream result;
  result << "DrakeSystem/" << setfill('0') << setw(16) << hex << get_next_id();
  return result.str();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::System)
