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
#include "drake/systems/rendering/pose_bundle.h"

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
using multibody::Body;
using multibody::BodyIndex;
using multibody::MultibodyTree;
using multibody::MultibodyPlant;
using multibody::PositionKinematicsCache;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::SpatialAcceleration;
using multibody::SpatialForce;
using multibody::SpatialInertia;
using multibody::UniformGravityFieldElement;
using multibody::UnitInertia;
using multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;
using systems::OutputPort;
using systems::State;

template<typename T>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    double mass, double length, double radius, int num_links,
    geometry::GeometrySystem<T>* geometry_system) :
    MultibodyPlant<T>("NLinkPendulumPlant"),
    mass_(mass), length_(length), radius_(radius), num_links_(num_links) {
    this->Init(geometry_system);
}

// TODO: read scalar conversion's doc on how to do this for non-final systems
// like MultibodyPlant.
#if 0
template<typename T>
template<typename U>
NLinkPendulumPlant<T>::NLinkPendulumPlant(
    const NLinkPendulumPlant<U>& other) :
    NLinkPendulumPlant<T>(
        other.get_mass(),
        other.get_length(),
        other.get_radius(),
        other.get_num_links(), nullptr) {
}
#endif

template<typename T>
void NLinkPendulumPlant<T>::BuildMultibodyModel(
    multibody::MultibodyTree<T>* model) {

  // Evenly distribute mass across the n-links.
  double link_mass = get_mass() / get_num_links();
  double link_length = get_length() / get_num_links();

  // Define each link's spatial inertia about its center of mass Bcm.
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::SolidCylinder(
          get_radius(), link_length, Vector3<double>::UnitY());
  SpatialInertia<double> M_Bcm(link_mass, Vector3<double>::Zero(), G_Bcm);

  // A pointer to the last added link.
  const RigidBody<T>* previous_link = &model->get_world_body();

  std::stringstream stream;

  // Add rigid bodies for each link.
  for (int link_index = 0; link_index < get_num_links(); ++link_index) {
    const RigidBody<T>& link = model->template AddBody<RigidBody>(M_Bcm);
    // Create a joint name.
    stream.clear();
    stream << "Joint_" << link_index;
    // Pose of F frame attached on previous link P, measured in P.
    Isometry3d X_PF(Translation3d(0.0, -link_length / 2.0, 0.0));
    // Pose of M frame attached on the new link B, measured in B.
    Isometry3d X_BM(Translation3d(0.0, link_length / 2.0, 0.0));

    const RevoluteJoint<T>& joint =
        model->template AddJoint<RevoluteJoint>(stream.str(),
        *previous_link, X_PF, link, X_BM, Vector3d::UnitZ());
    joints_.push_back(&joint);

    previous_link = &link;
  }
  DRAKE_ASSERT(model->get_num_bodies() == get_num_links() + 1);
  DRAKE_ASSERT(model->get_num_joints() == get_num_links());

  model->template AddForceElement<UniformGravityFieldElement>(
      Vector3d(0.0, -9.81, 0.0));

  model->Finalize();
}

template<typename T>
void NLinkPendulumPlant<T>::DoRegisterGeometry(
    geometry::GeometrySystem<T>* geometry_system) const {
  // skip the world.
  for (BodyIndex body_index(1);
       body_index < this->get_num_bodies(); ++body_index) {
    const Body<T>& body = this->get_model().get_body(body_index);
    this->RegisterGeometry(
        body,
        /* pose of the geometry in the body frame. */
        Isometry3<double>::Identity(),
        std::make_unique<Sphere>(get_radius()),geometry_system);
  }
}

template<typename T>
void NLinkPendulumPlant<T>::SetStraightAtAnAngle(
    Context<T>* context, const T& angle) const {
  joints_[0]->set_angle(context, angle);
  //for (const RevoluteJoint<T>* joint : joints_) {
  //  joint->set_angle(context, angle);
  //}
}

template class NLinkPendulumPlant<double>;
//template class NLinkPendulumPlant<AutoDiffXd>;

}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake
