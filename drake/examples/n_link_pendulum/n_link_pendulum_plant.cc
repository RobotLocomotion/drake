#include "drake/examples/n_link_pendulum/n_link_pendulum_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace n_link_pendulum {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::Sphere;
using multibody::PositionKinematicsCache;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UniformGravityFieldElement;
using multibody::UnitInertia;
using multibody::VelocityKinematicsCache;
using systems::Context;

template<typename T>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    double mass, double length, double radius, int num_links,
    geometry::GeometrySystem<T>* geometry_system) :
    mass_(mass), length_(length), radius_(radius), num_links_(num_links) {
  // Build the MultibodyTree model of this plant.
  BuildMultibodyTreeModel(geometry_system);
  DRAKE_DEMAND(model_.get_num_positions() == num_links);
  DRAKE_DEMAND(model_.get_num_velocities() == num_links);
  DRAKE_DEMAND(model_.get_num_states() == 2 * num_links);

  this->DeclareContinuousState(
      model_.get_num_positions(),
      model_.get_num_velocities(), 0 /* num_z */);

  // The geometry system pointer can be nullptr to signify:
  //   - we are not interested in visualization and/or geometric queries.
  //   - This is being invoked from the scalar-conversion constructor.
  if (geometry_system != nullptr) {
    // Build the GeometrySystem model of this plant.
    source_id_ = geometry_system->RegisterSource("n_link_pendulum");
    geometry_id_port_ =
        this->DeclareAbstractOutputPort(
                &NLinkPendulumPlant::AllocateFrameIdOutput,
                &NLinkPendulumPlant::CalcFrameIdOutput).get_index();

#if 0
    geometry_pose_port_ =
        this->DeclareAbstractOutputPort(&NLinkPendulumPlant::AllocateFramePoseOutput,
                                        &NLinkPendulumPlant::CalcFramePoseOutput)
            .get_index();

    AllocateGeometry(geometry_system);
#endif
  }
}

template <typename T>
FrameIdVector NLinkPendulumPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_.is_valid());
  FrameIdVector ids(source_id_);
  ids.AddFrameIds(body_ids_);
  return ids;
}

template <typename T>
void NLinkPendulumPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}


template<typename T>
template<typename U>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    const NLinkPendulumPlant<U>& other) :
    NLinkPendulumPlant<T>(
        other.get_mass(),
        other.get_length(),
        other.get_radius(),
        other.get_num_links(), nullptr) {
  // Make a copy of GeometrySystem-specific info.
  source_id_ = other.source_id_;
}

template<typename T>
void NLinkPendulumPlant<T>::BuildMultibodyTreeModel(
    geometry::GeometrySystem<T>* geometry_system) {

  // Evenly distribute mass across the n-links.
  double link_mass = get_mass() / get_num_links();
  double link_length = get_length() / get_num_links();

  // Define each link's spatial inertia about its center of mass Bcm.
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::SolidCylinder(
          get_radius(), link_length, Vector3<double>::UnitY());
  SpatialInertia<double> M_Bcm(link_mass, Vector3<double>::Zero(), G_Bcm);

  body_ids_.reserve(get_num_links());

  // A pointer to the last added link.
  const RigidBody<T>* previous_link = &model_.get_world_body();

  std::stringstream stream;

  // Add rigid bodies for each link.
  for (int link_index = 0; link_index < get_num_links(); ++link_index) {
    const RigidBody<T>& link = model_.template AddBody<RigidBody>(M_Bcm);
    // Create a joint name.
    stream.clear();
    stream << "Joint_" << link_index;
    // Pose of F frame attached on previous link P, measured in P.
    Isometry3d X_PF(Translation3d(0.0, -link_length / 2.0, 0.0));
    // Pose of M frame attached on the new link B, measured in B.
    Isometry3d X_BM(Translation3d(0.0, link_length / 2.0, 0.0));

    const RevoluteJoint<T>& joint =
        model_.template AddJoint<RevoluteJoint>(stream.str(),
        *previous_link, X_PF, link, X_BM, Vector3d::UnitZ());
    joints_.push_back(&joint);

    // Define the geometry for each link
    if (geometry_system != nullptr) {
      // Earth - Earth's frame is at the center of the sun.
      FrameId link_frame_id = geometry_system->RegisterFrame(
          source_id_, GeometryFrame("Earth", Isometry3<double>::Identity()));
      body_ids_.push_back(link_frame_id);

      // The geometry is right at the frame's origin.
      geometry_system->RegisterGeometry(
          source_id_, link_frame_id,
          std::make_unique<GeometryInstance>(
              Isometry3<double>::Identity(),
              std::make_unique<Sphere>(get_radius())));
    }

    previous_link = &link;
  }
  DRAKE_ASSERT(model_.get_num_bodies() == get_num_links());
  DRAKE_ASSERT(model_.get_num_joints() == get_num_links());

  model_.template AddForceElement<UniformGravityFieldElement>(
      Vector3d(0.0, -9.81, 0.0));

  model_.Finalize();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
NLinkPendulumPlant<T>::DoMakeContext() const {
  return model_.CreateDefaultContext();
}

template<typename T>
void NLinkPendulumPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = model_.get_num_velocities();

  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  // Check if M is symmetric.
  const T err_sym = (M - M.transpose()).norm();
  DRAKE_DEMAND(err_sym < 10 * std::numeric_limits<double>::epsilon());

  VectorX<T> C(nv);
  model_.CalcBiasTerm(context, &C);

  auto v = x.bottomRows(nv);

  VectorX<T> xdot(model_.get_num_states());
  xdot << v, M.llt().solve(-C);
  derivatives->SetFromVector(xdot);
}

template<typename T>
T NLinkPendulumPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  // TODO: make this an MBT method.
  // Use it also in DoCalcTimeDerivatives()
  const auto& mbt_context =
      dynamic_cast<const multibody::MultibodyTreeContext<T>&>(context);
  Eigen::VectorBlock<const VectorX<T>> v = mbt_context.get_velocities();

  const int nv = model_.get_num_velocities();

  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  return 0.5 * v.transpose() * M * v;
}

template<typename T>
T NLinkPendulumPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  return model_.CalcPotentialEnergy(context);
}

template class NLinkPendulumPlant<double>;
template class NLinkPendulumPlant<AutoDiffXd>;

}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake
