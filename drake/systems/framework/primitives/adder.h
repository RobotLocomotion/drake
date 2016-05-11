#pragma once

#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many inputs of equal length.
/// @tparam T The type of mathematical object being added.
/// TODO(david-german-tri): Implement DiscreteSystemInterface.
template <typename T>
class Adder : public ContinuousSystemInterface<T> {
 public:
  /// @param num_inputs is the number of input ports to be added.
  /// @param length is the size of each input port.
  Adder(int num_inputs, int length);
  virtual ~Adder() {}

  /// Allocates the number of input ports specified in the constructor.
  /// Allocates no state.
  Context<T> CreateDefaultContext() const override;

  /// Allocates one output port of the width specified in the constructor.
  SystemOutput<T> CreateDefaultOutput() const override;

  /// Sums the input ports into the output port. If the input ports are not
  /// of number num_inputs_ or size length_, std::runtime_error will be thrown.
  void Output(const Context<T>& context, Cache<T>* cache,
              SystemOutput<T>* output) const override;

  /// Returns an empty vector since this System is stateless.
  void GetDerivativesOfGeneralizedPosition(
      const Context<T>& context, Cache<T>* cache,
      VectorInterface<T>* derivatives) const override;

  /// Returns an empty vector since this System is stateless.
  void GetDerivativesOfGeneralizedVelocity(
      const Context<T>& context, Cache<T>* cache,
      VectorInterface<T>* derivatives) const override;

  /// Returns an empty vector since this System is stateless.
  void GetDerivativesOfOtherContinuousState(
      const Context<T>& context, Cache<T>* cache,
      VectorInterface<T>* derivatives) const override;

  /// Returns an empty vector since this System is stateless.
  void MapVelocityToConfigurationDerivative(
      const Context<T>& context, Cache<T>* cache,
      VectorInterface<T>* derivatives) const override;

  std::string get_name() const override { return "adder"; }

 private:
  const int num_inputs_;
  const int length_;
};

}  // namespace systems
}  // namesapce drake
