#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_string(
    print_resource_path, "",
    "Given the relative path of a resource within Drake, e.g., "
    "`drake/examples/pendulum/Pendulum.urdf`, "
    "find the resource and print its absolute path, e.g., "
    "`/home/user/tmp/drake/examples/pendulum/Pendulum.urdf`");
DEFINE_bool(
    print_resource_root_environment_variable_name, false,
    "Print the name of the environment variable that provides the "
    "first place where this tool attempts to look. This flag cannot be used "
    "in combination with the other flags.");
DEFINE_string(
    add_resource_search_path, "",
    "Adds path in which the resources live. This directory will be"
    "searched after the environment variable but before the directory in which"
    " `.drake-resource-sentinel` is, if such a directory is found. This flag "
    "can be used in combination with `print_resource_path`");

namespace drake {
namespace {

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Find Drake-related resources");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  // Allowed flag combinations (num_commands value):
  // FLAGS_print_resource_path (1)
  // FLAGS_print_resource_path + FLAGS_add_resource_search_path (1)
  // FLAGS_print_resource_root_environment_variable_name (1)
  const int num_commands =
      (FLAGS_print_resource_path.empty() ? 0 : 1) +
      (FLAGS_print_resource_root_environment_variable_name ? 1 : 0) +
      (!FLAGS_add_resource_search_path.empty() &&
               FLAGS_print_resource_root_environment_variable_name
           ? 1
           : 0);
  if (num_commands != 1) {
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }

  if (FLAGS_print_resource_root_environment_variable_name) {
    std::cout << drake::kDrakeResourceRootEnvironmentVariableName << "\n";
    return 0;
  }

  if (!FLAGS_add_resource_search_path.empty()) {
    AddResourceSearchPath(FLAGS_add_resource_search_path);
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
