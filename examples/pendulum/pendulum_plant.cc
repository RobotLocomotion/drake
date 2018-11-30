#include "drake/examples/pendulum/pendulum_plant.h"

#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace pendulum {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::Cylinder;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::MakeDrakeVisualizerProperties;
using geometry::Sphere;
using std::make_unique;

template <typename T>
PendulumPlant<T>::PendulumPlant()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<pendulum::PendulumPlant>{}) {
  this->DeclareVectorInputPort(PendulumInput<T>());
  state_port_ = this->DeclareVectorOutputPort(PendulumState<T>(),
                                              &PendulumPlant::CopyStateOut)
                    .get_index();
  this->DeclareContinuousState(PendulumState<T>(), 1 /* num_q */, 1 /* num_v */,
                               0 /* num_z */);
  this->DeclareNumericParameter(PendulumParams<T>());
}

template <typename T>
template <typename U>
PendulumPlant<T>::PendulumPlant(const PendulumPlant<U>& p) : PendulumPlant() {
  source_id_ = p.source_id();
  frame_id_ = p.frame_id();

  if (source_id_.is_valid()) {
    geometry_pose_port_ = AllocateGeometryPoseOutputPort();
  }
}

template <typename T>
PendulumPlant<T>::~PendulumPlant() {}

template <typename T>
const systems::InputPort<T>& PendulumPlant<T>::get_input_port() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::OutputPort<T>& PendulumPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

template <typename T>
const systems::OutputPort<T>& PendulumPlant<T>::get_geometry_pose_output_port()
    const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
void PendulumPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    PendulumState<T>* output) const {
  output->set_value(get_state(context).get_value());
}

template <typename T>
void PendulumPlant<T>::CopyPoseOut(const systems::Context<T>& context,
                                   geometry::FramePoseVector<T>* poses) const {
  DRAKE_DEMAND(poses->size() == 1);
  DRAKE_DEMAND(poses->source_id() == source_id_);

  const T theta = get_state(context).theta();

  poses->clear();
  Isometry3<T> pose = Isometry3<T>::Identity();
  pose.linear() = math::RotationMatrix<T>::MakeYRotation(theta).matrix();
  poses->set_value(frame_id_, pose);
}

template <typename T>
T PendulumPlant<T>::CalcTotalEnergy(const systems::Context<T>& context) const {
  using std::pow;
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params = get_parameters(context);
  // Kinetic energy = 1/2 m l² θ̇ ².
  const T kinetic_energy =
      0.5 * params.mass() * pow(params.length() * state.thetadot(), 2);
  // Potential energy = -mgl cos θ.
  const T potential_energy =
      -params.mass() * params.gravity() * params.length() * cos(state.theta());
  return kinetic_energy + potential_energy;
}

// Compute the actual physics.
template <typename T>
void PendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const PendulumState<T>& state = get_state(context);
  const PendulumParams<T>& params = get_parameters(context);
  PendulumState<T>& derivative_vector = get_mutable_state(derivatives);

  derivative_vector.set_theta(state.thetadot());
  derivative_vector.set_thetadot(
      (get_tau(context) -
       params.mass() * params.gravity() * params.length() * sin(state.theta()) -
       params.damping() * state.thetadot()) /
      (params.mass() * params.length() * params.length()));
}

template <typename T>
void PendulumPlant<T>::RegisterGeometry(
    const PendulumParams<double>& params,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(!source_id_.is_valid());
  DRAKE_DEMAND(scene_graph);

  source_id_ = scene_graph->RegisterSource("pendulum");

  // The base.
  GeometryId id = scene_graph->RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(Isometry3d(Translation3d(0., 0., .025)),
                                    make_unique<Box>(.05, 0.05, 0.05), "base"));
  scene_graph->AssignRole(
      source_id_, id, MakeDrakeVisualizerProperties(Vector4d(.3, .6, .4, 1)));

  frame_id_ = scene_graph->RegisterFrame(
      source_id_, GeometryFrame("arm", Isometry3d::Identity()));

  // The arm.
  id = scene_graph->RegisterGeometry(
      source_id_, frame_id_,
      make_unique<GeometryInstance>(
          Isometry3d(Translation3d(0, 0, -params.length() / 2.)),
          make_unique<Cylinder>(0.01, params.length()), "arm"));
  scene_graph->AssignRole(
      source_id_, id, MakeDrakeVisualizerProperties(Vector4d(.9, .1, 0, 1)));

  // The mass at the end of the arm.
  id = scene_graph->RegisterGeometry(
      source_id_, frame_id_,
      make_unique<GeometryInstance>(
          Isometry3d(Translation3d(0, 0, -params.length())),
          make_unique<Sphere>(params.mass() / 40.), "arm point mass"));
  scene_graph->AssignRole(
      source_id_, id, MakeDrakeVisualizerProperties(Vector4d(0, 0, 1, 1)));

  // Now allocate the output port.
  geometry_pose_port_ = AllocateGeometryPoseOutputPort();
}

template <typename T>
systems::OutputPortIndex PendulumPlant<T>::AllocateGeometryPoseOutputPort() {
  DRAKE_DEMAND(source_id_.is_valid() && frame_id_.is_valid());
  return this->DeclareAbstractOutputPort("geometry_pose",
      geometry::FramePoseVector<T>(source_id_, {frame_id_}),
                                  &PendulumPlant<T>::CopyPoseOut)
      .get_index();
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::pendulum::PendulumPlant)
