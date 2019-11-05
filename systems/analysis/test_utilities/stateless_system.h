#pragma once

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System with no state for testing a simplistic witness function.
template <class T>
class StatelessSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StatelessSystem)

  StatelessSystem(double offset, const WitnessFunctionDirection& dir_type)
      : LeafSystem<T>(SystemTypeTag<StatelessSystem>{}),
        offset_(offset) {
    witness_ = this->MakeWitnessFunction(
        "clock witness", dir_type, &StatelessSystem::CalcClockWitness,
        &StatelessSystem::InvokePublishCallback);
  }

  /// Scalar-converting copy constructor. See @ref system_scalar_conversion.
  /// @note This function does not preserve the publish callback because
  ///       this is test code for which it is expected that no one will care
  ///       whether the publish callback survives transmogrification.
  template <typename U>
  explicit StatelessSystem(const StatelessSystem<U>& other)
      : StatelessSystem<T>(other.get_trigger_time(),
                           other.witness_->direction_type()) {}

  void set_publish_callback(
      std::function<void(const Context<T>&)> callback) {
    publish_callback_ = callback;
  }

  /// Gets the time that the witness function triggered.
  double get_trigger_time() const { return offset_; }

 protected:
  void DoGetWitnessFunctions(
      const Context<T>&,
      std::vector<const WitnessFunction<T>*>* w) const override {
    w->push_back(witness_.get());
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class StatelessSystem;

  // The witness function is the time value itself less the offset value.
  T CalcClockWitness(const Context<T>& context) const {
    return context.get_time() - offset_;
  }

  void InvokePublishCallback(const Context<T>& context,
                             const PublishEvent<T>&) const {
    if (this->publish_callback_ != nullptr) this->publish_callback_(context);
  }

  const double offset_;
  std::unique_ptr<WitnessFunction<T>> witness_;
  std::function<void(const Context<T>&)> publish_callback_{nullptr};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
