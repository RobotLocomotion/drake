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

/** Multiply the single vector input by a scalar and present the result as
the single output port.
@tparam T The type of numerical values processed here. Must be a
          valid Eigen scalar. **/
template <typename T>
class Gain3 : public System3<T> {
 public:
  /** Create %Gain System with one input and one output port of the given
  length. **/
  // TODO(sherm1) Shouldn't specify the length; any size is OK.
  // TODO(sherm1) Gain should be a parameter.
  DRAKESYSTEMFRAMEWORK_EXPORT Gain3(const std::string& name, const T& gain,
                                    int length);

  /** Report the gain that was supplied in the constructor. **/
  T get_gain() const { return gain_; }

  /** Change the gain from its current value. **/
  void set_gain(const T& gain) { gain_ = gain; }

 private:
  // Implement System<T> interface.

  std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const override {
    return std::make_unique<Context3<T>>();
  }

  // Set output to the product of the gain and the input port.
  DRAKESYSTEMFRAMEWORK_EXPORT void DoCalcOutputPort(
      const AbstractContext3& context, int port_num,
      AbstractValue* value) const override;

  T gain_;
  const int length_;
};

}  // namespace systems
}  // namespace drake
