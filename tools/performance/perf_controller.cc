#include "drake/tools/performance/perf_controller.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "drake/common/never_destroyed.h"

namespace drake {
namespace tools {
namespace performance {

namespace {

constexpr const char enable_cmd[] = "enable";
constexpr int enable_len = sizeof(enable_cmd);
constexpr const char disable_cmd[] = "disable";
constexpr int disable_len = sizeof(disable_cmd);
constexpr const char ack_cmd[] = "ack\n";
constexpr int ack_len = sizeof(ack_cmd);

int FdFromEnv(const char* env_key) {
  const char* name = std::getenv(env_key);
  if (name == nullptr) {
    return -1;
  }
  return open(name, O_RDWR);
}

}  // namespace

PerfController::PerfController() {
  ctl_fd_ = FdFromEnv("DRAKE_PERF_CTL_FIFO");
  ack_fd_ = FdFromEnv("DRAKE_PERF_ACK_FIFO");
  is_control_available_ = (ctl_fd_ >= 0 && ack_fd_ >= 0);
}

void PerfController::pause() {
  if (!is_control_available()) {
    return;
  }
  send_command(disable_cmd, disable_len);
}

void PerfController::resume() {
  if (!is_control_available()) {
    return;
  }
  send_command(enable_cmd, enable_len);
}

bool PerfController::is_control_available() const {
  return is_control_available_;
}

void PerfController::send_command(const char* command, int command_len) {
  if (!is_control_available()) {
    return;
  }
  int wrote_bytes = write(ctl_fd_, command, command_len);
  if (wrote_bytes < command_len) {
    throw std::runtime_error("`perf` command not completely written.");
  }
  char ack[5];
  int read_bytes = read(ack_fd_, ack, ack_len);
  if (read_bytes < ack_len) {
    throw std::runtime_error(
        "`perf` command acknowledgment not completely read.");
  }
  if (std::strcmp(ack, ack_cmd) != 0) {
    throw std::runtime_error("`perf` command acknowledgment not received.");
  }
}

PerfController& ThePerfController() {
  static never_destroyed<PerfController> controller;
  return controller.access();
}

}  // namespace performance
}  // namespace tools
}  // namespace drake
