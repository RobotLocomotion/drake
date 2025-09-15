/* clang-format off */
#include "drake/lcm/drake_lcm_log.h"
/* clang-format on */

#include <memory>
#include <source_location>
#include <stdexcept>
#include <string>

namespace drake {
namespace lcm {
namespace {
[[noreturn]] void ThrowError(
    std::source_location caller = std::source_location::current()) {
  throw std::logic_error(fmt::format(
      "{} cannot be used because the LCM runtime library has been disabled "
      "in this build of Drake",
      caller.function_name()));
}
}  // namespace

class DrakeLcmLog::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);
};

DrakeLcmLog::DrakeLcmLog(const std::string&, bool is_write,
                         bool overwrite_publish_time_with_system_clock)
    : is_write_{/* ignored */},
      overwrite_publish_time_with_system_clock_{/* ignored */} {
  ThrowError();
}

DrakeLcmLog::~DrakeLcmLog() = default;

std::string DrakeLcmLog::get_lcm_url() const {
  ThrowError();
}

void DrakeLcmLog::Publish(const std::string&, const void*, int,
                          std::optional<double>) {
  ThrowError();
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmLog::Subscribe(
    const std::string&, HandlerFunction) {
  ThrowError();
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmLog::SubscribeMultichannel(
    std::string_view, MultichannelHandlerFunction) {
  ThrowError();
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmLog::SubscribeAllChannels(
    MultichannelHandlerFunction) {
  ThrowError();
}

int DrakeLcmLog::HandleSubscriptions(int) {
  ThrowError();
}

double DrakeLcmLog::GetNextMessageTime() const {
  ThrowError();
}

void DrakeLcmLog::DispatchMessageAndAdvanceLog(double) {
  ThrowError();
}

void DrakeLcmLog::OnHandleSubscriptionsError(const std::string&) {
  ThrowError();
}

bool DrakeLcmLog::available() {
  return false;
}

}  // namespace lcm
}  // namespace drake
