#include "drake/systems/framework/system.h"

#include <iomanip>
#include <ios>
#include <regex>

#include <fmt/format.h>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"

namespace {

// Output a string like "System::EvalInput()".
std::string FmtFunc(const char* func) {
  return fmt::format("System::{}()", func);
}

}

namespace drake {
namespace systems {

template <typename T>
void System<T>::ThrowNegativeInputPortIndex(const char* func,
                                            int port_index) const {
  DRAKE_DEMAND(port_index < 0);
  throw std::out_of_range(
      fmt::format("{}: negative port index {} is illegal. (Subsystem {})",
                  FmtFunc(func), port_index, GetSystemPathname()));
}

template <typename T>
void System<T>::ThrowInputPortIndexOutOfRange(const char* func,
                                              InputPortIndex port,
                                              int num_input_ports) const {
  DRAKE_DEMAND(num_input_ports >= 0);
  throw std::out_of_range(
      fmt::format("{}: there is no input port with index {} because there "
                  "are only {} input ports in subsystem {}.",
                  FmtFunc(func), port, num_input_ports, GetSystemPathname()));
}

template <typename T>
void System<T>::ThrowInputPortHasWrongType(
    const char* func, InputPortIndex port, const std::string& expected_type,
    const std::string& actual_type) const {
  throw std::logic_error(fmt::format(
      "{}: expected value of type {} for input port[{}] "
      "but the actual type was {}. (Subsystem {})",
      FmtFunc(func), expected_type, port, actual_type, GetSystemPathname()));
}

template <typename T>
void System<T>::ThrowCantEvaluateInputPort(const char* func,
                                           InputPortIndex port) const {
  throw std::logic_error(
      fmt::format("{}: input port[{}] is neither connected nor freestanding so "
                  "cannot be evaluated. (Subsystem {})",
                  FmtFunc(func), port, GetSystemPathname()));
}

std::string SystemImpl::GetMemoryObjectName(
    const std::string& nice_type_name, int64_t address) {
  using std::setfill;
  using std::setw;
  using std::hex;

  // Remove the template parameter(s).
  const std::string type_name_without_templates = std::regex_replace(
      nice_type_name, std::regex("<.*>$"), std::string());

  // Replace "::" with "/" because ":" is System::GetSystemPathname's separator.
  // TODO(sherm1) Change the separator to "/" and avoid this!
  const std::string default_name = std::regex_replace(
      type_name_without_templates, std::regex(":+"), std::string("/"));

  // Append the address spelled like "@0123456789abcdef".
  std::ostringstream result;
  result << default_name << '@' << setfill('0') << setw(16) << hex << address;
  return result.str();
}

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class System<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class System<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::System)
