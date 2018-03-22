#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant_sdf.h"
#include "drake/multibody/benchmarks/acrobot/acrobot_sdf.h"

#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

using Eigen::Vector3d;

using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeAcrobotPlantSdf(bool finalize) {

  auto plant = std::make_unique<MultibodyPlant<double>>();

  // Load the SDF string
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(multibody::benchmarks::acrobotSdf);

  // Check for any errors.
  if (!errors.empty()) {
    std::cerr << "Errors loading SDF string.\n";
    for (auto e : errors)
      std::cerr << e << std::endl;
    return plant;
  }

  // Get a pointer to the first model.
  const sdf::Model *model = root.ModelByIndex(0);
  if (!model) {
    std::cerr << "No models present in SDF string.";
    return plant;
  }

  // Add all the links
  for (uint64_t linkIndex = 0; linkIndex < model->LinkCount(); ++linkIndex)
  {
    const sdf::Link *link = model->LinkByIndex(linkIndex);

    // Get the SDF inertia
    const ignition::math::Inertiald &inertiaSdf = link->Inertial();

    // Create the multibody inertia
    // \todo(nkoenig) Write test to verify that I'm setting this correctly.
    SpatialInertia<double> inertia =
      SpatialInertia<double>::MakeFromCentralInertia(
          inertiaSdf.MassMatrix().Mass(),
          Vector3d(inertiaSdf.Pose().Pos().X(),
                   inertiaSdf.Pose().Pos().Y(),
                   inertiaSdf.Pose().Pos().Z()),
          UnitInertia<double>(
            inertiaSdf.MassMatrix().DiagonalMoments().X(),
            inertiaSdf.MassMatrix().DiagonalMoments().Y(),
            inertiaSdf.MassMatrix().DiagonalMoments().Z(),
            inertiaSdf.MassMatrix().OffDiagonalMoments().X(),
            inertiaSdf.MassMatrix().OffDiagonalMoments().Y(),
            inertiaSdf.MassMatrix().OffDiagonalMoments().Z()));

    // Add a rigid body to model each link.
    plant->AddRigidBody(link->Name(), inertia);
  }

  // Add all the joints
  for (uint64_t jointIndex = 0; jointIndex < model->JointCount();
       ++jointIndex) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint *joint = model->JointByIndex(jointIndex);
    const sdf::JointAxis *axis = joint->Axis();

    // Only supporting revolute joints for now.
    if (joint->Type() == sdf::JointType::REVOLUTE) {
      /// \todo(nkoenig) Check that the links exist before creating the
      /// joint.

      // Special case for a joint connected to the world.
      if (joint->ParentLinkName().empty() ||
          joint->ParentLinkName() == "world") {
        plant->AddJoint<RevoluteJoint>(joint->Name(),
            plant->world_body(), {},
            plant->GetBodyByName(joint->ChildLinkName()), {},
            Vector3d(axis->Xyz().X(),axis->Xyz().Y(),axis->Xyz().Z()));
      } else {
        plant->AddJoint<RevoluteJoint>(joint->Name(),
            plant->GetBodyByName(joint->ParentLinkName()), {},
            plant->GetBodyByName(joint->ChildLinkName()), {},
            Vector3d(axis->Xyz().X(),axis->Xyz().Y(),axis->Xyz().Z()));
      }
    }
  }

  // Add acrobot's actuator at the elbow joint.
  plant->AddJointActuator("ElbowActuator",
      plant->GetJointByName("ElbowJoint"));

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3d::UnitZ());

  if (finalize)
    plant->Finalize();

  return plant;
}

}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
