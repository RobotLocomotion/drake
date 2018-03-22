#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant_sdf.h"

#include <memory>

#include <sdf/sdf.hh>

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
MakeAcrobotPlantSdf() {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  // Load the SDF string
  sdf::Root root;
  sdf::Errors errors = root.LoadSdfString(
    "<?xml version='1.0' ?>"
    "<sdf version='1.6'>"
    "  <model name='acrobot'>"
    "    <link name='Link1'>"
    "      <inertial>"
    "        <pose>0 0 0.5 0 0 0</pose>"
    "        <mass>1.0</mass>"
    "        <!-- This inertia is based on a solid cylinder with"
    "             radius=0.001 meters and height=1.0 meters. -->"
    "        <inertia>"
    "          <ixx>0.083</ixx><iyy>0.083</iyy><izz>0.0000005</izz>"
    "          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
    "        </inertia>"
    "      </inertial>"
    "    </link>"
    "    <link name='Link2'>"
    "      <inertial>"
    "        <pose>0 0 0.5 0 0 0</pose>"
    "        <mass>1.0</mass>"
    "        <!-- This inertia is based on a solid cylinder with"
    "             radius=1.0 meters and height=1.0 meters. -->"
    "        <inertia>"
    "          <ixx>0.33</ixx><iyy>0.33</iyy><izz>0.5</izz>"
    "          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
    "        </inertia>"
    "      </inertial>"
    "    </link>"
    "    <joint name='ShoulderJoint' type='revolute'>"
    "      <parent>world</parent>"
    "      <child>Link1</child>"
    "      <axis>"
    "        <xyz>1.0 0 0</xyz>"
    "      </axis>"
    "    </joint>"
    "    <joint name='ElbowJoint' type='revolute'>"
    "      <parent>Link1</parent>"
    "      <child>Link2</child>"
    "      <axis>"
    "        <xyz>1.0 0 0</xyz>"
    "      </axis>"
    "    </joint>"
    "  </model>"
    "</sdf>");

  // Check for any errors.
  if (!errors.empty()) {
    std::string error_accumulation;
    for (const auto& e : errors)
      error_accumulation += e.Message();
    throw std::runtime_error(error_accumulation);
  }

  // Get a pointer to the first model.
  const sdf::Model* model = root.ModelByIndex(0);
  if (!model) {
    throw std::logic_error("No models present in SDF string.");
  }

  // Add all the links
  for (uint64_t link_index = 0; link_index < model->LinkCount(); ++link_index) {
    const sdf::Link* link = model->LinkByIndex(link_index);

    // Get the link's inertia relative to the Bcm frame.
    const ignition::math::Inertiald& I_Bcm_B = link->Inertial();

    // The axes of the inertial reference frame do not need to be aligned
    // with the principal axes of inertia.
    const ignition::math::Matrix3d I_BBcm_Bi = I_Bcm_B.MOI();

    // Create the multibody inertia
    // TODO(nkoenig) Write test to verify that I'm setting this correctly.
    SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::MakeFromCentralInertia(
          I_Bcm_B.MassMatrix().Mass(),
          Vector3d(I_Bcm_B.Pose().Pos().X(),
                   I_Bcm_B.Pose().Pos().Y(),
                   I_Bcm_B.Pose().Pos().Z()),
          RotationalInertia<double>(
            I_BBcm_Bi(0, 0), I_BBcm_Bi(1, 1), I_BBcm_Bi(2, 2),
            I_BBcm_Bi(1, 0), I_BBcm_Bi(2, 0), I_BBcm_Bi(2, 1)));

    // Add a rigid body to model each link.
    plant->AddRigidBody(link->Name(), M_BBo_B);
  }

  // Add all the joints
  for (uint64_t joint_index = 0; joint_index < model->JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint* joint = model->JointByIndex(joint_index);
    const sdf::JointAxis* axis = joint->Axis();

    // Only supporting revolute joints for now.
    if (joint->Type() == sdf::JointType::REVOLUTE) {
      // TODO(nkoenig) Check that the links exist before creating the
      /// joint.

      // Special case for a joint connected to the world.
      if (joint->ParentLinkName().empty() ||
          joint->ParentLinkName() == "world") {
        plant->AddJoint<RevoluteJoint>(joint->Name(),
            plant->world_body(), {},
            plant->GetBodyByName(joint->ChildLinkName()), {},
            Vector3d(axis->Xyz().X(), axis->Xyz().Y(), axis->Xyz().Z()));
      } else {
        plant->AddJoint<RevoluteJoint>(joint->Name(),
            plant->GetBodyByName(joint->ParentLinkName()), {},
            plant->GetBodyByName(joint->ChildLinkName()), {},
            Vector3d(axis->Xyz().X(), axis->Xyz().Y(), axis->Xyz().Z()));
      }
    }
  }

  // Gravity acting in the -z direction.
  // TODO(nkoenig) A model in SDF has no knowledge about external forces,
  // such as gravity. We would have to place the model inside a <world>
  // and assign a <gravity>.
  plant->AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3d::UnitZ());

  plant->Finalize();

  return plant;
}

}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
