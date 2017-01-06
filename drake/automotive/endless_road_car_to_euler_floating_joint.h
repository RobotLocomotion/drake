#pragma once

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Converts EndlessRoadCarState to a full 6-DOF EulerFloatingJointState,
/// using the RoadGeometry on which the car is (presumably) driving.
template <typename T>
class EndlessRoadCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  // TODO(maddog)  Is there a nice way for this to get the RoadGeometry from
  //               the connected car?  (It will be a moot question once the
  //               complete ROAD-state including Lane* is used.)
  EndlessRoadCarToEulerFloatingJoint(
      const maliput::utility::InfiniteCircuitRoad* road)
      : road_(road) {
    this->set_name("EndlessRoadCarToEulerFloatingJoint");
    this->DeclareInputPort(systems::kVectorValued,
                           EndlessRoadCarStateIndices::kNumCoordinates,
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
    const EndlessRoadCarState<T>* const input_data =
        dynamic_cast<const EndlessRoadCarState<T>*>(input_vector);
    DRAKE_ASSERT(input_data != nullptr);

    Base* const output_vector = output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);
    EulerFloatingJointState<T>* const output_data =
        dynamic_cast<EulerFloatingJointState<T>*>(output_vector);
    DRAKE_ASSERT(output_data != nullptr);

    maliput::api::LanePosition lp(input_data->s(), input_data->r(), 0.);
    maliput::api::GeoPosition geo = road_->lane()->ToGeoPosition(lp);
    output_data->set_x(geo.x);
    output_data->set_y(geo.y);
    output_data->set_z(geo.z);

    // Simple treatment of orientation:  "car always points in the direction
    // of its velocity vector", e.g., no slip, oversteeer, etc.
    // Hence, we express forward-orientation of the car as a composition of
    // two rotations:  rotation in sr-plane to align with velocity vector,
    // followed by srh->xyz rotation of LANE-space orientation.
    maliput::api::Rotation rot = road_->lane()->GetOrientation(lp);
    // TODO(maddog)  Deal with (sigma_dot < 0).
    const double theta = input_data->heading();

    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    const double ca = std::cos(rot.roll);
    const double sa = std::sin(rot.roll);
    const double cb = std::cos(rot.pitch);
    const double sb = std::sin(rot.pitch);
    const double cg = std::cos(rot.yaw);
    const double sg = std::sin(rot.yaw);

    const double A = ct*(cb*cg) + st*(-ca*sg + sa*sb*cg);
    const double B = ct*(cb*sg) + st*(ca*cg + sa*sb*sg);
    const double C = -ct*(sb) + st*(sa*cb);
    const double D = st*(sb) + ct*(sa*cb);
    const double E = ca*cb;
    // TODO(maddog)  yaw and roll are only correct when pitch is in the
    //               range [-PI/2, PI/2] --- do we care about flipped cars?
    output_data->set_yaw(std::atan2(B, A));
    output_data->set_pitch(std::atan2(-C, std::sqrt(D*D + E*E)));
    output_data->set_roll(std::atan2(D, E));
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }

 private:
  const maliput::utility::InfiniteCircuitRoad* road_;
};

}  // namespace automotive
}  // namespace drake
