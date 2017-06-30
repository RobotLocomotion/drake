#include "drake/systems/framework/system.h"

#include <iomanip>
#include <ios>
#include <regex>

namespace drake {
namespace systems {

std::string SystemImpl::GetMemoryObjectName(
    const std::string& nice_type_name, int64_t address) {
  using std::setfill;
  using std::setw;
  using std::hex;

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

template class System<double>;

}  // namespace systems
}  // namespace drake
