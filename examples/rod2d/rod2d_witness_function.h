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
  Rod2dWitnessFunction(const Rod2D<T>* rod,
                       systems::WitnessFunctionDirection dir,
                       RodEndpoint endpoint) :
      systems::WitnessFunction<T>(rod, dir),
      rod_(rod),
      endpoint_(endpoint) {
    event_ = std::make_unique<systems::UnrestrictedUpdateEvent<T>>(
      systems::Event<T>::TriggerType::kWitness);
  }

  /// Gets the rod.
  const Rod2D<T>& get_rod() const { return *rod_; }

  /// Gets the endpoint that this witness function tracks.
  int get_endpoint() const { return endpoint_; }

 private:
  // Unique pointer to the event.
  std::unique_ptr<systems::UnrestrictedUpdateEvent<T>> event_;

  // Pointer to the rod system.
  const Rod2D<T>* rod_{nullptr};

  // The endpoint of the rod that this witness function tracks.
  RodEndpoint endpoint_{kInvalid};
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake

