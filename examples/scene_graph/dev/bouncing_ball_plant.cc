#include "drake/examples/scene_graph/dev/bouncing_ball_plant.h"

#include <algorithm>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using std::make_unique;
using systems::Context;
using systems::Value;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        SceneGraph<T>* scene_graph,
                                        const Vector2<double>& p_WB)
    : source_id_(source_id), p_WB_(p_WB) {
  DRAKE_DEMAND(scene_graph != nullptr);
  DRAKE_DEMAND(source_id_.is_valid());

  geometry_query_port_ = this->DeclareAbstractInputPort(
      systems::kUseDefaultName, systems::Value<geometry::QueryObject<T>>{})
           .get_index();
  state_port_ =
      this->DeclareVectorOutputPort(BouncingBallVector<T>(),
                                    &BouncingBallPlant::CopyStateToOutput)
          .get_index();

  this->DeclareContinuousState(BouncingBallVector<T>(), 1 /* num_q */,
                               1 /* num_v */, 0 /* num_z */);
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");

  ball_frame_id_ = scene_graph->RegisterFrame(
      source_id, GeometryFrame("ball_frame", Isometry3<double>::Identity()));
  ball_id_ = scene_graph->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(), /*X_FG*/
                                    make_unique<Sphere>(diameter_ / 2.0),
                                    "ball"));
  // Use the default material.
  scene_graph->AssignRole(source_id, ball_id_, IllustrationProperties());
  scene_graph->AssignRole(source_id, ball_id_, ProximityProperties());

  // Allocate the output port now that the frame has been registered.
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          FramePoseVector<double>(source_id_, {ball_frame_id_}),
          &BouncingBallPlant::CalcFramePoseOutput)
      .get_index();
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::InputPort<T>&
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
void BouncingBallPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_DEMAND(poses->source_id() == source_id_);
  DRAKE_DEMAND(poses->size() == 1);

  Isometry3<T> pose = Isometry3<T>::Identity();
  const BouncingBallVector<T>& state = get_state(context);
  pose.translation() << p_WB_.x(), p_WB_.y(), state.z();
  poses->clear();
  poses->set_value(ball_frame_id_, pose);
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
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake
