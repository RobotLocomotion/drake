#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/context3.h"
#include "drake/systems/framework/system3.h"
#include "drake/systems/framework/system3_output.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// An adder for arbitrarily many vector-valued inputs of equal length.
/// @tparam T The type of numerical values processed here. Must be a
///           valid Eigen scalar.
template <typename T>
class Adder3 : public System3<T> {
 public:
  /** Create an Adder System with a given number of VectorInputPort's of
  a particular length, with a single VectorOutputPort of the same length.
  @param num_inputs is the number of input ports to be added.
  @param length is the size of each input port. **/
  DRAKESYSTEMFRAMEWORK_EXPORT Adder3(const std::string& name, int num_inputs,
                                     int length);

 private:
  // Implement System3<T> interface.

  // Allocates the number of input ports specified in the constructor.
  // Allocates no state.
  std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const override {
    return std::make_unique<Context3<T>>();
  }

  // Sums the input ports into the output port. If the input ports are not
  // of number num_inputs_ or size length_, std::runtime_error will be thrown.
  DRAKESYSTEMFRAMEWORK_EXPORT void DoCalcOutputPort(
      const AbstractContext3& context, int port_num,
      AbstractValue* value) const override;

  const int num_inputs_;
  const int length_;
};

}  // namespace systems
}  // namespace drake
