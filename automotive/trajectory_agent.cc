#include "drake/automotive/trajectory_agent.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace automotive {

using multibody::SpatialVelocity;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

template <typename T>
TrajectoryAgent<T>::TrajectoryAgent(const AgentData& agent_data,
                                    const AgentTrajectory& trajectory,
                                    double sampling_time_sec)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::TrajectoryAgent>{}),
      agent_data_(agent_data),
      trajectory_(trajectory) {
  this->DeclarePeriodicUnrestrictedUpdate(sampling_time_sec, 0.);
  this->DeclareVectorOutputPort(&TrajectoryAgent::CalcStateOutput);
  this->DeclareVectorOutputPort(&TrajectoryAgent::CalcPoseOutput);
  this->DeclareVectorOutputPort(&TrajectoryAgent::CalcVelocityOutput);
}

template <typename T>
void TrajectoryAgent<T>::CalcStateOutput(
    const systems::Context<T>& context,
    SimpleCarState<T>* output_vector) const {
  ImplCalcOutput(GetValues(context), output_vector);
}

template <typename T>
void TrajectoryAgent<T>::CalcPoseOutput(const systems::Context<T>& context,
                                        PoseVector<T>* pose) const {
  ImplCalcPose(GetValues(context), pose);
}

template <typename T>
void TrajectoryAgent<T>::CalcVelocityOutput(const systems::Context<T>& context,
                                            FrameVelocity<T>* velocity) const {
  ImplCalcVelocity(GetValues(context), velocity);
}

template <typename T>
void TrajectoryAgent<T>::ImplCalcOutput(const PoseVelocity& values,
                                        SimpleCarState<T>* output) const {
  output->set_x(T{values.pose3().x()});
  output->set_y(T{values.pose3().y()});
  output->set_heading(T{values.pose3().z()});
  output->set_velocity(T{values.speed()});
}

template <typename T>
void TrajectoryAgent<T>::ImplCalcPose(const PoseVelocity& values,
                                      PoseVector<T>* pose) const {
  pose->set_translation(Eigen::Translation<T, 3>{values.translation()});
  const Eigen::Quaternion<double>& q =
      math::RotationMatrix<double>(values.rotation()).ToQuaternion();
  pose->set_rotation(Eigen::Quaternion<T>{q});
}

template <typename T>
void TrajectoryAgent<T>::ImplCalcVelocity(const PoseVelocity& values,
                                          FrameVelocity<T>* velocity) const {
  const Eigen::Vector3d& v = values.velocity().translational();
  const Eigen::Vector3d& w = values.velocity().rotational();
  velocity->set_velocity(SpatialVelocity<T>{Vector3<T>{w}, Vector3<T>{v}});
}

template <typename T>
PoseVelocity TrajectoryAgent<T>::GetValues(
    const systems::Context<T>& context) const {
  return trajectory_.value(ExtractDoubleOrThrow(context.get_time()));
}

}  // namespace automotive
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::TrajectoryAgent)
