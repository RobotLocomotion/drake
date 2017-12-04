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

template <class T>
class LogisticSystem;

/// Witness function for determining when the state of the logistic system
/// crosses zero.
template <class T>
class LogisticWitness : public systems::WitnessFunction<T> {
 public:
  ~LogisticWitness() override {}
  explicit LogisticWitness(const LogisticSystem<T>& system) :
    systems::WitnessFunction<T>(
        system,
        systems::WitnessFunctionDirection::kCrossesZero) {
  }

 protected:
  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    events->add_publish_event(
        std::make_unique<PublishEvent<T>>(Event<T>::TriggerType::kWitness));
  }

  // The witness function is simply the state value itself.
  T DoEvaluate(const Context<T>& context) const override {
    return context.get_continuous_state()[0];
  }
};

/// System with state evolution yielding a logistic function, for purposes of
/// witness function testing using the differential equation
/// dx/dt = α⋅(1 - (x/k)^ν)⋅t, where ν > 0 (affects the shape of the curve),
/// α > 0 (growth rate), and k is the upper asymptote.
template <class T>
class LogisticSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LogisticSystem)

  LogisticSystem(double k, double alpha, double nu) : k_(k), alpha_(alpha),
      nu_(nu) {
    this->DeclareContinuousState(1);
    witness_ = std::make_unique<LogisticWitness<T>>(*this);
  }

  void set_publish_callback(
      std::function<void(const Context<double>&)> callback) {
    publish_callback_ = callback;
  }

 protected:
  void DoCalcTimeDerivatives(const systems::Context<T>& context,
      systems::ContinuousState<T>* continuous_state) const override {
    using std::pow;

    // Get the current time.
    const T& t = context.get_time();

    // Get state.
    const T& x = context.get_continuous_state()[0];

    // Compute the derivative.
    (*continuous_state)[0] = alpha_ * (1 - pow(x/k_, nu_)) * t;
  }

  void DoGetWitnessFunctions(
      const systems::Context<T>&,
      std::vector<const systems::WitnessFunction<T>*>* w) const override {
    w->push_back(witness_.get());
  }

  void DoPublish(
      const drake::systems::Context<double>& context,
      const std::vector<const systems::PublishEvent<double>*>&) const override {
    if (publish_callback_ != nullptr) publish_callback_(context);
  }

 private:
  std::unique_ptr<LogisticWitness<T>> witness_;
  std::function<void(const Context<double>&)> publish_callback_{nullptr};

  // The upper asymptote on the logistic function.
  double k_{1.0};

  // The rate (> 0) at which the logistic function approaches the asymptote.
  double alpha_{1.0};

  // Parameter (> 0) that affects near which asymptote maximum growth occurs.
  double nu_{1.0};
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
