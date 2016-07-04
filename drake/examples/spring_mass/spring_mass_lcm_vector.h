#pragma once

// TODO(liang.fok) Automatically generate this file.

#include "drake/examples/spring_mass/lcm_vector_interface.h"
#include "lcmtypes/drake/lcmt_spring_mass_state_t.hpp"

namespace drake {
namespace examples {
namespace spring_mass {

/// A concrete implementation of `drake::systems::LCMVectorInterface` that
/// specializies it to the spring-mass LCM state message.
template <typename T>
class SpringMassLCMVector :
    public drake::systems::LCMVectorInterface<T, lcmt_spring_mass_state_t> {
 public:
  /// The state space is size 2: [position, velocity].
  static const int kStateSize = 2;

  /// The constructor initilizes the state vector to be NaN and the timestamp to
  /// be -1.
  SpringMassLCMVector() : timestamp_(-1), values_(VectorX<T>::Constant(
            kStateSize, std::numeric_limits<
                      typename Eigen::NumTraits<T>::Real>::quiet_NaN())) {}

  /// The destructor.
  virtual ~SpringMassLCMVector() {}

  int size() const override {
    return kStateSize;
  }

  void set_value(const Eigen::Ref<const VectorX<T>>& value) override {
    if (value.size() != 2)
      throw std::runtime_error("SpringMassLCMVector: set_value: ERROR: "
        "Attempted to set value using a vector of length not equal to two.");
    values_ = value;
  }

  virtual int64_t get_timestamp() const {
    return timestamp_;
  }

  Eigen::VectorBlock<const VectorX<T>> get_value() const override {
    return values_.head(values_.rows());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_value() override {
    return values_.head(values_.rows());
  }

  std::unique_ptr<drake::systems::VectorInterface<T>> Clone() const override {
    return std::unique_ptr<SpringMassLCMVector<T>>(DoClone());
  }

  void Encode(const lcmt_spring_mass_state_t& message) override {
    timestamp_ = message.timestamp;
    values_[0] = message.position;
    values_[1] = message.velocity;
  }

  void Decode(lcmt_spring_mass_state_t* message) override {
    message->timestamp = timestamp_;
    message->position = values_[0];
    message->velocity = values_[1];
  }

 private:
  // Returns a new SpringMassLCMVector containing a copy of the entire state.
  virtual SpringMassLCMVector<T>* DoClone() const {
    SpringMassLCMVector* clone = new SpringMassLCMVector();
    clone->values_ = values_;
    return clone;
  }

  int64_t timestamp_{};
  VectorX<T> values_;
};

}  // namespace spring_mass
}  // namespace examples
}  // namesapce drake
