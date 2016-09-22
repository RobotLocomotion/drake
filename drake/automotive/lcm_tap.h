#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include <lcm/lcm-cpp.hpp>

#include "drake/systems/vector.h"

namespace drake {
namespace automotive {

/// Implements a Drake System (@see drake/systems/System.h) that
/// publishes its input to LCM, but otherwise merely passes through
/// its input to output, and has no state.
template <template <typename> class Vector>
class LcmTap {
 public:
  /// Create an LcmTap that publishes on the given @p lcm instance.
  explicit LcmTap(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm) {}

  // Noncopyable.
  LcmTap(const LcmTap&) = delete;
  LcmTap& operator=(const LcmTap&) = delete;

  /// @name Implement the Drake System concept.
  //@{

  template <typename ScalarType>
  using StateVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Vector<ScalarType>;

  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const {
    return StateVector<double>();
  }

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const {
    typename Vector<double>::LCMMessageType msg;
    if (!encode(t, u, msg)) {
      throw std::runtime_error(std::string("failed to encode") +
                               msg.getTypeName());
    }
    lcm_->publish(u.channel(), &msg);
    return u;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return true; }

  //@}

 private:
  const std::shared_ptr<lcm::LCM> lcm_;
};

}  // namespace automotive
}  // namespace drake
