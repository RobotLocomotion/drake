#include <iostream>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"

// This program is a helper stub for initialization_sequence_test.py.

DEFINE_string(lcm_url, "", "");

namespace drake {
namespace lcm {
namespace test {
namespace {

void PauseForInput() {
  std::string dummy;
  std::getline(std::cin, dummy);
}

int main() {
  const bool defer_initialization = true;
  DrakeLcm dut(FLAGS_lcm_url, defer_initialization);
  Subscriber<lcmt_drake_signal> subscriber(&dut, "CHANNEL_NAME");

  log()->info("recv_parts_test_stub: construction is complete");
  PauseForInput();

  dut.HandleSubscriptions(0);
  log()->info("recv_parts_test_stub: activation is complete");
  PauseForInput();

  return 0;
}

}  // namespace
}  // namespace test
}  // namespace lcm
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::lcm::test::main();
}
