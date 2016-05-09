#pragma once

#include "drake/systems/framework/continuous_system_interface.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many inputs of equal length.
template <typename ScalarType>
class Adder : public ContinuousSystemInterface<ScalarType> {
 public:
  /// @param num_inputs is the number of input ports to be added.
  /// @param length is the size of each input port.
  Adder(int num_inputs, int length);
  virtual ~Adder() {}

  /// Allocates the number of input ports specified in the constructor.
  /// Allocates no state.
  Context<ScalarType> CreateDefaultContext() const override;

  /// Allocates one output port of the width specified in the constructor.
  SystemOutput<ScalarType> CreateDefaultOutput() const override;

  /// Sums the input ports into the output port. If the input ports are not
  /// of number num_inputs_ or size length_, std::runtime_error will be thrown.
  void Output(const Context<ScalarType>& context, Cache<ScalarType>* cache,
              SystemOutput<ScalarType>* output) const override;

  /// Returns an empty vector since this System is stateless.
  void GetContinuousDerivativesOfGeneralizedVelocity(
      const Context<ScalarType>& context, Cache<ScalarType>* cache,
      VectorInterface<ScalarType>* derivatives) const override;

  /// Returns an empty vector since this System is stateless.
  void GetContinuousDerivativesOfGeneralizedPosition(
      const Context<ScalarType>& context, Cache<ScalarType>* cache,
      VectorInterface<ScalarType>* derivatives) const override;

  /// Returns an empty vector since this System is stateless.
  void MapVelocityToConfigurationDerivative(
      const VectorInterface<ScalarType>& generalized_velocity,
      Cache<ScalarType>* cache,
      VectorInterface<ScalarType>* derivatives) const override;

  std::string get_name() const override { return "adder"; }

 private:
  const int num_inputs_;
  const int length_;
};

}  // namespace systems
}  // namesapce drake
