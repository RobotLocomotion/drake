#pragma once

#include <cstdint>
#include <memory>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context3.h"
#include "drake/systems/framework/system3.h"
#include "drake/systems/framework/system3_output.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/** This System produces a constant VectorInterface value on its single
OutputPort.
@tparam T The type of numerical values processed here. Must be a
          valid Eigen scalar. **/
template <typename T>
class VectorConstant3 : public System3<T> {
 public:
  /** Takes over ownership of the given VectorInterface, which will
  be regurgitated as the value of the OutputPort. **/
  VectorConstant3(const std::string& name,
                  std::unique_ptr<VectorInterface<T>> value)
      : System3<T>(name) {
    // Provide this as the "model value" for the output port.
    auto port = std::make_unique<VectorOutputPort3<T>>(std::move(value));
    AddOutputPort(std::move(port));
  }

  /** Given an Eigen vector value, create a BasicVector containing that
  value as the output of this system. **/
  VectorConstant3(const std::string& name,
                  const Eigen::Ref<const VectorX<T>>& vector)
    : VectorConstant3(name, MakeBasicVector(vector)) {}

  /** Given a scalar, create a one-element BasicVector containing that
  value as the output of this system. **/
  VectorConstant3(const std::string& name,
                  const T& scalar_value)
    : VectorConstant3(name, Vector1<T>::Constant(scalar_value)) {}

 private:
  static std::unique_ptr<BasicVector<T>> MakeBasicVector(
      const Eigen::Ref<const VectorX<T>>& vector) {
    auto basic_vector = std::make_unique<BasicVector<T>>((int)vector.rows());
    basic_vector->get_mutable_value() = vector;
    return basic_vector;
  }

  // Implement System<T> interface.

  std::unique_ptr<AbstractContext3> DoCreateEmptyContext() const override {
    return std::make_unique<Context3<T>>();
  }

  // Don't need any additional resources.

  // Calculation consists just of copying the model value to the output.
  void DoCalcOutputPort(const AbstractContext3& /*context*/,
                        int port_num, AbstractValue* result) const override {
    *result = get_output_port(port_num).get_model_value();
  }
};

}  // namespace systems
}  // namespace drake
