/*
Provides a C++ version libsdformat's `gz sdf` ruby command-line interface.
*/

#include <stdexcept>
#include <string>

#include <gflags/gflags.h>
#include <sdf/config.hh>
// XXX This include path spelling is terrible; can we do better??
#include "drake_src/src/cmd/gz.hh"

#include "drake/common/drake_assert.h"

DEFINE_string(check, "", "Checks an SDFormat file");
DEFINE_string(describe, "", "Describes a given SDFormat version");
DEFINE_string(
    print, "",
    "Prints an SDFormat file after parsing and converting to the converted "
    "to the latest specification");

namespace drake {
namespace {

namespace cmd_gz = drake_vendor::sdf::SDF_VERSION_NAMESPACE;
using cmd_gz::cmdCheck;
using cmd_gz::cmdDescribe;
using cmd_gz::cmdPrint;

int DoMain() {
  // All flags are mutually exclusive.
  const int num_top_level_options =
      !FLAGS_check.empty() + !FLAGS_print.empty() + !FLAGS_describe.empty();
  if (num_top_level_options != 1) {
    throw std::runtime_error("Must provide exactly one of the given options");
  }

  if (!FLAGS_check.empty()) {
    return cmdCheck(FLAGS_check.c_str());
  } else if (!FLAGS_describe.empty()) {
    return cmdDescribe(FLAGS_describe.c_str());
  } else if (!FLAGS_print.empty()) {
    return cmdPrint(FLAGS_print.c_str(), 0, 0, 0, 0, -1, 0);
  }
  DRAKE_UNREACHABLE();
}

}  // namespace
}  // namespace drake

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::DoMain();
}
