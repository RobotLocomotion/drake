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

namespace drake {
namespace {

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Find Drake-related resources");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  if (FLAGS_print_resource_path.empty()) {
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
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
