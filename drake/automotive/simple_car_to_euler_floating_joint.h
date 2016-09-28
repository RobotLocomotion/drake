#pragma once

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Converts SimpleCarState to a full 6-DOF EulerFloatingJointState.
template <typename T>
class SimpleCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  SimpleCarToEulerFloatingJoint() {
    this->set_name("SimpleCarToEulerFloatingJoint");
    this->DeclareInputPort(systems::kVectorValued,
                           SimpleCarStateIndices::kNumCoordinates,
                           systems::kContinuousSampling);
    this->DeclareOutputPort(systems::kVectorValued,
                            EulerFloatingJointStateIndices::kNumCoordinates,
                            systems::kContinuousSampling);
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
    DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

    typedef systems::VectorBase<T> Base;
    const Base* const input_vector = this->EvalVectorInput(context, 0);
    DRAKE_ASSERT(input_vector != nullptr);
    const SimpleCarState<T>* const input_data =
        dynamic_cast<const SimpleCarState<T>*>(input_vector);
    DRAKE_ASSERT(input_data != nullptr);

    Base* const output_vector = output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);
    EulerFloatingJointState<T>* const output_data =
        dynamic_cast<EulerFloatingJointState<T>*>(output_vector);
    DRAKE_ASSERT(output_data != nullptr);

    output_data->set_x(input_data->x());
    output_data->set_y(input_data->y());
    output_data->set_z(0.0);
    output_data->set_roll(0.0);
    output_data->set_pitch(0.0);
    output_data->set_yaw(input_data->heading());
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }
};

}  // namespace automotive
}  // namespace drake
