#pragma once

#include <memory>

#include "drake/examples/rod2d/rod2d_endpoint.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace rod2d {

template <class T>
class Rod2D;

/// All witness functions for the 2D Rod inherit from this one.
template <class T>
class Rod2dWitnessFunction : public systems::WitnessFunction<T> {
 public:
  /// Constructs the witness function for the given rod with specified witness
  /// function direction and endpoint to be tracked.
  Rod2dWitnessFunction(const Rod2D<T>& rod,
                       systems::WitnessFunctionDirection dir,
                       RodEndpoint endpoint) :
      systems::WitnessFunction<T>(rod, dir),
      rod_(rod),
      endpoint_(endpoint) {
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
  }

  /// Gets whether the witness function is active (default). If it is not
  /// active, it will not be used to track state changes.
  bool is_active() const { return active_; }

  /// Sets whether the witness function is active.
  void set_active(bool flag) { active_ = flag; }

  /// Gets the rod.
  const Rod2D<T>& get_rod() const { return rod_; }

  /// Gets the endpoint that this witness function tracks.
  int get_endpoint() const { return endpoint_; }

  /// The types of witness function.
  enum class WitnessType {
      /// The signed distance for a contact from the half-space.
      kSignedDistance,

      /// The acceleration along the contact normal at a point of contact.
      kNormalAccel,

      /// The velocity along the contact normal at a point of contact.
      kNormalVel,

      /// The slack in the stiction forces. If the slack is non-zero, stiction
      /// will be maintained. When it is less than zero, too much stiction
      /// force is being generated.
      kStickingFrictionForceSlack,

      /// The normal force applied at a point of contact.
      kNormalForce,

      /// The witness function for detecting transitions between non-sliding
      /// and sliding contact.
      kSlidingWitness,
  };

  /// Gets the type of witness function.
  virtual WitnessType get_witness_function_type() const = 0;

 private:
  void DoAddEvent(systems::CompositeEventCollection<T>* events) const override {
    event_->set_attribute(
        std::make_unique<systems::Value<const Rod2dWitnessFunction<T>*>>(this));
    event_->add_to_composite(events);
  }

  // Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;

  // Reference to the rod system.
  const Rod2D<T>& rod_;

  // The endpoint of the rod that this witness function tracks.
  RodEndpoint endpoint_{kInvalid};

  /// Whether the witness function is used to track state changes.
  bool active_{true};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

