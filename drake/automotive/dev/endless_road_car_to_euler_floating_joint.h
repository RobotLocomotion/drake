#pragma once

#include <memory>

#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/gen/endless_road_car_state.h"
#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {
// TODO(maddog@tri.global)  EndlessRoadCarState should be generalized to
//                          RoadCarState, which packs a maliput::api::Lane*
//                          in addition to (s, r), and this class would
//                          generalize accordingly.

/// Converts EndlessRoadCarState to a full 6-DOF EulerFloatingJointState
/// in the world frame, mapping via the RoadGeometry on which the car is
/// (presumably) driving.
template <typename T>
class EndlessRoadCarToEulerFloatingJoint : public systems::LeafSystem<T> {
 public:
  /// Constructor.
  ///
  /// @param road  the InfiniteCircuitRoad which defines the mapping from
  ///              LANE-space (s, r) coordinates to world-frame position
  ///              and orientation.
  EndlessRoadCarToEulerFloatingJoint(
      const maliput::utility::InfiniteCircuitRoad* road)
      : road_(road) {
    this->set_name("EndlessRoadCarToEulerFloatingJoint");
    // Single EndlessRoadCarState input.
    this->DeclareInputPort(systems::kVectorValued,
                           EndlessRoadCarStateIndices::kNumCoordinates);
    // Single EulerFloatingJointState output.
    this->DeclareOutputPort(systems::kVectorValued,
                            EulerFloatingJointStateIndices::kNumCoordinates);
  }

  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override {
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
    // of its heading", e.g., no slip, oversteeer, etc.  Hence, we express
    // forward-orientation of the car as a composition of two rotations:
    //  1) rotation in lane's sr-plane to account for the heading,
    const double theta = input_data->heading();
    // followed by:
    //  2) srh->xyz orientation of LANE-space frame w.r.t. world frame.
    maliput::api::Rotation rot = road_->lane()->GetOrientation(lp);

    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    const double ca = std::cos(rot.roll);
    const double sa = std::sin(rot.roll);
    const double cb = std::cos(rot.pitch);
    const double sb = std::sin(rot.pitch);
    const double cg = std::cos(rot.yaw);
    const double sg = std::sin(rot.yaw);

    // TODO(maddog@tri.global)  Replace this hand-rolled math with rotmat2rpy()
    //                          applied to the underlying product of rotations.
    // We compute the five coefficients of the composite rotation matrix
    // which we need to extract roll/pitch/yaw.
    const double A = ct*(cb*cg) + st*(-ca*sg + sa*sb*cg);
    const double B = ct*(cb*sg) + st*(ca*cg + sa*sb*sg);
    const double C = -ct*(sb) + st*(sa*cb);
    const double D = st*(sb) + ct*(sa*cb);
    const double E = ca*cb;
    // NB: Yaw and roll are only correct when pitch is in the range
    //     [-PI/2, PI/2]; this doesn't handle flipped roads/cars.
    output_data->set_yaw(std::atan2(B, A));
    output_data->set_pitch(std::atan2(-C, std::sqrt(D*D + E*E)));
    output_data->set_roll(std::atan2(D, E));
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::OutputPortDescriptor<T>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<T>>();
  }

 private:
  const maliput::utility::InfiniteCircuitRoad* const road_;
};

}  // namespace automotive
}  // namespace drake
