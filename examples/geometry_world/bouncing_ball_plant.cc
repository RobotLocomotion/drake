#include "drake/examples/geometry_world/bouncing_ball_plant.h"

#include <algorithm>

#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace examples {
namespace geometry_world {
namespace bouncing_ball {

using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::PenetrationAsPointPair;
using geometry::SourceId;
using geometry::Sphere;
using systems::Context;
using systems::Value;
using std::make_unique;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        GeometrySystem<T>* geometry_system,
                                        const Vector2<double>& p_WB)
    : source_id_(source_id), p_WB_(p_WB) {
  DRAKE_DEMAND(geometry_system != nullptr);

  geometry_query_port_ = this->DeclareAbstractInputPort().get_index();
  state_port_ =
      this->DeclareVectorOutputPort(BouncingBallVector<T>(),
                                    &BouncingBallPlant::CopyStateToOutput)
          .get_index();
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(&BouncingBallPlant::AllocateFrameIdOutput,
                                      &BouncingBallPlant::CalcFrameIdOutput)
          .get_index();
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
                                &BouncingBallPlant::AllocateFramePoseOutput,
                                &BouncingBallPlant::CalcFramePoseOutput)
                            .get_index();

  this->DeclareContinuousState(BouncingBallVector<T>(), 1 /* num_q */,
                               1 /* num_v */, 0 /* num_z */);
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");

  ball_frame_id_ = geometry_system->RegisterFrame(
      source_id, GeometryFrame("ball_frame",
                               Isometry3<double>::Identity()));
  ball_id_ = geometry_system->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(), /*X_FG*/
                                    make_unique<Sphere>(diameter_ / 2.0)));
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::InputPortDescriptor<T>&
BouncingBallPlant<T>::get_geometry_query_input_port() const {
  return systems::System<T>::get_input_port(geometry_query_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_geometry_id_output_port() const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_geometry_pose_output_port() const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

// Updates the state output port.
template <typename T>
void BouncingBallPlant<T>::CopyStateToOutput(
    const Context<T>& context,
    BouncingBallVector<T>* state_output_vector) const {
  state_output_vector->set_value(get_state(context).get_value());
}

template <typename T>
FramePoseVector<T> BouncingBallPlant<T>::AllocateFramePoseOutput() const {
  FramePoseVector<T> poses(source_id_);
  poses.mutable_vector().push_back(Isometry3<T>::Identity());
  return poses;
}

template <typename T>
void BouncingBallPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  const BouncingBallVector<T>& state = get_state(context);
  DRAKE_ASSERT(poses->vector().size() == 1);
  // This assumes *no* rotation, we only need to update the translation.
  poses->mutable_vector()[0].translation() << p_WB_(0),
      p_WB_(1), state.z();
  poses->mutable_vector()[0].linear() << Matrix3<T>::Identity();
}

template <typename T>
FrameIdVector BouncingBallPlant<T>::AllocateFrameIdOutput() const {
  return FrameIdVector(source_id_,
                       std::vector<geometry::FrameId>{ball_frame_id_});
}

template <typename T>
void BouncingBallPlant<T>::CalcFrameIdOutput(const systems::Context<T>&,
                                             FrameIdVector* frame_ids) const {
  *frame_ids = AllocateFrameIdOutput();
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BouncingBallVector<T>& state = get_state(context);
  BouncingBallVector<T>& derivative_vector = get_mutable_state(derivatives);

  const geometry::QueryObject<T>& query_object =
      this->EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryObject<T>>();

  std::vector<PenetrationAsPointPair<T>> penetrations =
      query_object.ComputePointPairPenetration();
  T fC = 0;  // the contact force
  if (penetrations.size() > 0) {
    for (const auto& penetration : penetrations) {
      if (penetration.id_A == ball_id_ || penetration.id_B == ball_id_) {
        // Penetration depth, > 0 during penetration.
        const T& x = penetration.depth;
        // Penetration rate, > 0 implies increasing penetration.
        const T& xdot = -state.zdot();

        fC = k_ * x * (1.0 + d_ * xdot);
      }
    }
  }
  derivative_vector.set_z(state.zdot());
  const T fN = max(0.0, fC);

  derivative_vector.set_zdot((-m_ * g_ + fN) / m_);
}

template class BouncingBallPlant<double>;

}  // namespace bouncing_ball
}  // namespace geometry_world
}  // namespace examples
}  // namespace drake
