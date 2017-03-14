#pragma once

#include <cmath>
#include <memory>

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/simple_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Converts SimpleCarState to a full 6-DOF EulerFloatingJointState.
template <typename T>
class SimpleCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleCarToEulerFloatingJoint)

  SimpleCarToEulerFloatingJoint() {
    this->set_name("SimpleCarToEulerFloatingJoint");
    this->DeclareVectorInputPort(SimpleCarState<T>());
    this->DeclareVectorOutputPort(EulerFloatingJointState<T>());
  }

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override {
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

    using std::cos;
    using std::sin;

    const double heading = input_data->heading();
    // TODO(liang.fok) The following number is hard-coded to equal the distance
    // between the origin and the middle of the rear axle in prius.sdf and
    // prius_with_lidar.sdf. Generalize this to support arbitrary models.
    const double p_MoVo{1.40948};
    output_data->set_x(p_MoVo * cos(heading) + input_data->x());
    output_data->set_y(p_MoVo * sin(heading) + input_data->y());
    output_data->set_z(0.0);
    output_data->set_roll(0.0);
    output_data->set_pitch(0.0);
    output_data->set_yaw(heading);
  }
};

}  // namespace automotive
}  // namespace drake
