#include "drake/examples/multibody/suction_gripper/suction_cup.h"

namespace drake {
namespace examples {
namespace multibody {
namespace suction_gripper {

template <typename T>
SuctionCup<T>::SuctionCup(const MultibodyPlant<T>* plant, BodyIndex body_index,
                          SuctionCupParameters parameters,
                          MultibodyElements multibody_elements)
    : plant_(plant),
      body_index_(body_index),
      parameters_{std::move(parameters)},
      multibody_elements_{multibody_elements} {
  // N.B. This system has not state. It is a feedthrough with its outputs
  // being completely determined from the inputs.

  // Input ports.
  suction_pressure_index_ =
      this->DeclareInputPort("suction_pressure", systems::kVectorValued, 1)
          .get_index();
  body_poses_index_ =
      this->DeclareAbstractInputPort("body_poses",
                                     Value<std::vector<RigidTransform<T>>>())
          .get_index();
  query_object_input_port_ =
      DeclareAbstractInputPort(
          "query_object", drake::Value<drake::geometry::QueryObject<double>>())
          .get_index();

  // Output ports.
  this->DeclareAbstractOutputPort(
      "spatial_forces", std::vector<ExternallyAppliedSpatialForce<T>>(),
      &SuctionCup<T>::CalcSpatialForces);
}

template <typename T>
MultibodyElements SuctionCup<T>::AddMultibodyPlantElements(
    BodyIndex body_index, const RigidTransform<double>& X_BC,
    const SuctionCupParameters& cup_parameters,
    MultibodyPlant<T>* plant) const {
  // Body, typically an end effector, on which the cup model is attached.
  const Body<T>& attachment = plant->get_body(body_index);

  // We use the model instance of "body_index"
  const ModelInstanceIndex cup_instance = attachment.model_instance();

  const T radius = cup_parameters.suction_rim_radius;
  const T height = cup_parameters.height;

  // Main cup body.
  // The main cup body is a particle (zero rotational inertia) with half the
  // mass of the total mass of the cup.
  const T cup_main_body_mass = cup_parameters.mass / 2.0;
  // Spatial inertia about the point mass' COM, i.e. p_BoBcm = 0.
  const SpatialInertia<T> M_B =
      SpatialInertia<T>::PointMass(cup_main_body_mass, Vector3<T>::Zero());
  const auto& cup_main_body =
      plant->AddRigidBody("cup_main_body", cup_instance, M_B);
  // We place the main body a the center of the cup.
  const RigidTransform<T> X_CMainBody(Vector3<T>(0.0, 0.0, height / 2.0));
  const RigidTransform<T> X_BMainBody = X_BC * X_CMainBody;
  // The cup's frame C z-axis points towards the cup.
  const auto& main_body_slider =
      plant->AddJoint<drake::multibody::PrismaticJoint>(
          "cup_main_body_slider", attachment, X_BMainBody, cup_main_body, {},
          Eigen::Vector3d::UnitZ(),
          /* lower limit */ 0,
          /* upper limit */ height / 2.0, cup_parameters.damping);
  // The total compliance of the cup is split between the main body and the
  // rim, so that the total effective stiffness is cup_parameters.stiffness.
  // The main body, with stiffness kₘ, is in series with the rim bodies. The
  // rim bodies in turn form a system of N springs in parallel, with stiffness
  // kᵣ. Therefore, for a cup of stiffness k, we have kₘ = kᵣ = 2k. Since each
  // rim spring is in parallel, its stiffness is kᵢ = kᵣ / N = 2k/N.

  // TODO: per spring damping so that the total effective damping is
  // cup_parameters.damping.

  const T main_body_stiffness = 2.0 * cup_parameters.stiffness;
  const T rim_particle_stiffness =
      2.0 * cup_parameters.stiffness / cup_parameters.num_nodes;

  // TODO: implement implicit springs in SAP. For now, replace with distance
  // constraint, which effectively models a spring damper, but it is
  // significantly more stable when using SAP.
  plant_->AddForceElement<drake::multibody::PrismaticSpring>(
      main_body_slider,
      /* nominal position */ 0.,
      /* stiffness */ rim_particle_stiffness);

  // Add bodies discretizing the cup's ring.
  // The ring entails half of the total mass, and therefore the mass of each
  // particle discretizing the ring is m/(2*N), where m is the mass of the cup
  // and N is the number of particles discretizing the ring.
  const T ring_particle_mass =
      cup_parameters.mass / 2.0 / cup_parameters.num_rim_nodes;
  const SpatialInertia<T> M_Bi =
      SpatialInertia<T>::PointMass(ring_particle_mass, Vector3<T>::Zero());
  for (int i = 0; i < cup_parameters.num_rim_nodes; ++i) {
    const base_name = "rim_particle_" + std::to_string(i);
    const auto& rim_particle =
        plant->AddRigidBody(base_name, cup_instance, M_Bi);

    // Per documentation, the cup is in the z-axis of the frame C defined by
    // the pose X_BC relative to the attachment body.
    const auto& rim_particle_slider =
        plant->AddJoint<drake::multibody::PrismaticJoint>(
            base_name + "_slider", cup_body, {}, rim_particle, {},
            Eigen::Vector3d::UnitZ(),
            /* lower limit */ 0,
            /* upper limit */ height / 2.0,
            /*damping*/ cup_parameters.damping);

    // TODO: implement implicit springs in SAP. For now, replace with distance
    // constraint, which effectively models a spring damper, but it is
    // significantly more stable when using SAP.
    plant_->AddForceElement<drake::multibody::PrismaticSpring>(
        rim_particle_slider,
        /* nominal position */ 0.,
        /* stiffness */ main_body_stiffness);

    // Contact geometry.
    // Rim particles are height/2 away from the main body along the z-axis.
    // Position of particle Bi in frame B.
    const T angle = 2.0 * M_PI / cup_parameters.num_rim_nodes * i;
    const Vector3<T> p_BBi(radius * cos(angle), radius * sin(angle),
                           height / 2.0);
    const RigidTransform<T> X_BBi(p_BBi);

    CoulombFriction<T> coulomb_friction(cup_parameters.rim_friction,
                                        cup_parameters.rim_friction);
    geometry::GometryId rim_particle_id = plant_->RegisterCollisionGeometry(
        rim_particle, X_BBi, drake::geometry::Sphere(0.0),
        base_name + "_geometry", coulomb_friction);
  }

  // Return information later on needed by this class to work with the newly
  // added multibody elements.
  MultibodyElements elements;
  elements.cup_body = cup_main_body.index();
  elements.rim_bodies.push_back(rim_particle_slider.index());
  elements.rim_contact_points.push_back(rim_particle_id);
  return elements;
}

template <typename T>
std::unique_ptr<SuctionCup<T>> SuctionCup<T>::MakeAndAddToPlant(
    MultibodyPlant<T>* plant, BodyIndex body_index,
    const RigidTransform<double>& X_BC, SuctionCupParameters cup_parameters) {
  MultibodyElements elements =
      AddMultibodyPlantElements(body_index, X_BC, cup_parameters, plant);

  // Make the cup system, fully described now by MultibodyElements.
  return std::make_unique<SuctionCup>(
      plant, body_index, std::move(cup_parameters), std::move(elements));
}

template <typename T>
SuctionCup<T>* AddToBuilder(systems::DiagramBuilder<T>* builder,
                            MultibodyPlant<T>* plant, BodyIndex body_index,
                            const RigidTransform<double>& X_BC,
                            SuctionCupParameters cup_parameters) {
  std::unique_ptr<SuctionCup<T>> owned_cup =
      MakeAndAddToPlant(palnt, body_index, X_BC, cup_parameters);
  return builder->AddSystem(std::move(owned_cup));
}

}  // namespace suction_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::SuctionCup)
