#include "drake/lcm/drake_lcm_base.h"

#include <stdexcept>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace lcm {
namespace {

[[noreturn]] void ThrowNotImplemented(const DrakeLcmBase& self,
                                      const char* func) {
  throw std::runtime_error(
      fmt::format("{}::{} is not implemented", NiceTypeName::Get(self), func));
}

}  // namespace

DrakeLcmBase::DrakeLcmBase() = default;

DrakeLcmBase::~DrakeLcmBase() = default;

std::string DrakeLcmBase::get_lcm_url() const {
  ThrowNotImplemented(*this, __func__);
}

void DrakeLcmBase::Publish(const std::string&, const void*, int,
                           std::optional<double>) {
  ThrowNotImplemented(*this, __func__);
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmBase::Subscribe(
    const std::string&, HandlerFunction) {
  ThrowNotImplemented(*this, __func__);
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmBase::SubscribeMultichannel(
    std::string_view, MultichannelHandlerFunction) {
  ThrowNotImplemented(*this, __func__);
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcmBase::SubscribeAllChannels(
    MultichannelHandlerFunction) {
  ThrowNotImplemented(*this, __func__);
}

int DrakeLcmBase::HandleSubscriptions(int) {
  ThrowNotImplemented(*this, __func__);
}

void DrakeLcmBase::OnHandleSubscriptionsError(const std::string&) {
  ThrowNotImplemented(*this, __func__);
}

}  // namespace lcm
}  // namespace drake
