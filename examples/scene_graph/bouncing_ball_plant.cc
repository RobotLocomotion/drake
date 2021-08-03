#include "drake/examples/scene_graph/bouncing_ball_plant.h"

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

using Eigen::Vector4d;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::PerceptionProperties;
using geometry::ProximityProperties;
using geometry::render::RenderLabel;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_unique;
using systems::Context;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        SceneGraph<T>* scene_graph,
                                        const Vector2<double>& p_WB)
    : p_WB_(p_WB) {
  DRAKE_DEMAND(scene_graph != nullptr);
  DRAKE_DEMAND(source_id.is_valid());

  geometry_query_port_ = this->DeclareAbstractInputPort(
      systems::kUseDefaultName, Value<geometry::QueryObject<T>>{})
          .get_index();
  auto state_index = this->DeclareContinuousState(
      BouncingBallVector<T>(), 1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
  state_port_ =
      this->DeclareStateOutputPort(systems::kUseDefaultName, state_index)
          .get_index();
  static_assert(BouncingBallVectorIndices::kNumCoordinates == 1 + 1, "");

  ball_frame_id_ = scene_graph->RegisterFrame(
      source_id, GeometryFrame("ball_frame"));
  ball_id_ = scene_graph->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance>(RigidTransformd::Identity(), /*X_FG*/
                                    make_unique<Sphere>(diameter_ / 2.0),
                                    "ball"));
  // Use the default material.
  scene_graph->AssignRole(source_id, ball_id_, IllustrationProperties());
  scene_graph->AssignRole(source_id, ball_id_, ProximityProperties());
  PerceptionProperties perception_properties;
  perception_properties.AddProperty("phong", "diffuse",
                                    Vector4d{0.8, 0.8, 0.8, 1.0});
  perception_properties.AddProperty("label", "id",
                                    RenderLabel(ball_id_.get_value()));
  scene_graph->AssignRole(source_id, ball_id_, perception_properties);

  // Allocate the output port now that the frame has been registered.
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          systems::kUseDefaultName,
          &BouncingBallPlant::CalcFramePoseOutput,
          {this->configuration_ticket()})
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

template <typename T>
void BouncingBallPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  RigidTransform<T> pose = RigidTransform<T>::Identity();
  const BouncingBallVector<T>& state = get_state(context);
  pose.set_translation({p_WB_.x(), p_WB_.y(), state.z()});
  *poses = {{ball_frame_id_, pose}};
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const BouncingBallVector<T>& state = get_state(context);
  BouncingBallVector<T>& derivative_vector = get_mutable_state(derivatives);

  const auto& query_object = get_geometry_query_input_port().
      template Eval<geometry::QueryObject<T>>(context);

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
