#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(
    print_resource_path, "",
    "Given the relative path of a resource within Drake, e.g., "
    "`drake/examples/Pendulum/Pendulum.urdf`, "
    "find the resource and print its absolute path, e.g., "
    "`/home/user/tmp/drake/examples/Pendulum/Pendulum.urdf`");
DEFINE_bool(
    print_resource_root_environment_variable_name, false,
    "Print the name of the environment variable that provides the "
    "first place where this tool attempts to look.");

namespace drake {
namespace {

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Find Drake-related resources");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  const int num_commands =
      (FLAGS_print_resource_path.empty() ? 0 : 1) +
      (FLAGS_print_resource_root_environment_variable_name ? 1 : 0);
  if (num_commands != 1) {
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  if (FLAGS_print_resource_root_environment_variable_name) {
    std::cout << drake::kDrakeResourceRootEnvironmentVariableName << "\n";
    return 0;
  }

  const FindResourceResult& result = FindResource(FLAGS_print_resource_path);
  if (result.get_absolute_path()) {
    std::cout << *result.get_absolute_path()
              << "\n";
    return 0;
  }

  std::cerr << "resource_tool: "
            << result.get_error_message().value()
            << "\n";
  return 1;
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::main(argc, argv);
}
