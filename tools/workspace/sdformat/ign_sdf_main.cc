/*
Provides a C++ version libsdformat's `ign sdf` ruby command-line interface.
*/

#include <stdexcept>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"

DEFINE_string(
    check, "",
    "Checks an SDFormat file");
DEFINE_string(
    describe, "",
    "Describes a given SDFormat version");
DEFINE_string(
    print, "",
    "Prints an SDFormat file after parsing and converting to the converted "
    "to the latest specification");

// These following `extern` functions are defined in ign.cc, and are normally
// consumed by Ruby in libsdformat as a shared library. However, we don't want
// Ruby, and the current header file (ign.hh) does not export all function, so
// we must declare them here.
// TODO(eric.cousineau): Remove these and include `ign.hh`. See:
// https://github.com/osrf/sdformat/issues/323
extern "C" int cmdCheck(const char *_path);
extern "C" int cmdDescribe(const char *_version);
extern "C" int cmdPrint(const char *_path);

namespace drake {
namespace {

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
    return cmdPrint(FLAGS_print.c_str());
  }
  DRAKE_UNREACHABLE();
}

}  // namespace
}  // namespace drake

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::DoMain();
}
