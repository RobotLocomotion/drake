#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant_sdf.h"

#include <memory>

#include <sdf/sdf.hh>

#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

// Unnamed namespace for free functions local to this file.
namespace {
// Helper function to express an ignition::math::Vector3d instance as
// a Vector3d instance.
Vector3d ToVector3(const ignition::math::Vector3d& vector) {
  return Vector3d(vector.X(), vector.Y(), vector.Z());
}

// Helper function to express an ignition::math::Pose3d instance as
// an Isometry3d instance.
Isometry3d ToIsometry3(const ignition::math::Pose3d& pose) {
  const Isometry3d::TranslationType translation(ToVector3(pose.Pos()));
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return translation * rotation;
}
}  // namespace

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
    "      <pose>0 0 -0.5 0 0 0</pose>"
    "      <inertial>"
    "        <mass>1.0</mass>"
    "        <!-- This inertia is based on a solid cylinder with"
    "             radius=0.001 meters and height=1.0 meters. -->"
    "        <inertia>"
    "          <ixx>0.083</ixx><iyy>0.083</iyy><izz>5e-7</izz>"
    "          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
    "        </inertia>"
    "      </inertial>"
    "    </link>"
    "    <link name='Link2'>"
    "      <pose>0 0 -2.0 0 0 0</pose>"
    "      <inertial>"
    "        <mass>1.0</mass>"
    "        <!-- This inertia is based on a solid cylinder with"
    "             radius=1.0 meters and height=2.0 meters. -->"
    "        <inertia>"
    "          <ixx>0.33</ixx><iyy>0.33</iyy><izz>5e-7</izz>"
    "          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
    "        </inertia>"
    "      </inertial>"
    "    </link>"
    "    <joint name='ShoulderJoint' type='revolute'>"
    "      <parent>world</parent>"
    "      <child>Link1</child>"
    "      <axis>"
    "        <xyz>0.0 1.0 0.0</xyz>"
    "      </axis>"
    "    </joint>"
    "    <joint name='ElbowJoint' type='revolute'>"
    "      <pose>0 0 -1.0 0 0 0</pose>"
    "      <parent>Link1</parent>"
    "      <child>Link2</child>"
    "      <axis>"
    "        <xyz>0.0 1.0 0.0</xyz>"
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
    // Inertial provides a representation for the SpatialInertia
    // M_Bcm_Bi of body B, about its center of mass Bcm, and expressed in an
    // inertial frame Bi as defined in <inertial> <pose></pose> </inertial>.
    // Per SDF specification, Bi's origin is at the COM Bcm, but Bi
    // is not necessarily aligned with B.
    const ignition::math::Inertiald& I_Bcm_Bi = link->Inertial();

    // The axes of the inertial reference frame do not need to be aligned
    // with the principal axes of inertia.
    // MOI() returns the rotational massmatrix of body B about the body's
    // COM, Bcm, expressed in the body's frame.
    const ignition::math::Matrix3d I_BBcm_B = I_Bcm_Bi.MOI();

    // Create the multibody inertia
    SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::MakeFromCentralInertia(
          I_Bcm_Bi.MassMatrix().Mass(),
          Vector3d(I_Bcm_Bi.Pose().Pos().X(),
                   I_Bcm_Bi.Pose().Pos().Y(),
                   I_Bcm_Bi.Pose().Pos().Z()),
          RotationalInertia<double>(
            I_BBcm_B(0, 0), I_BBcm_B(1, 1), I_BBcm_B(2, 2),
            I_BBcm_B(1, 0), I_BBcm_B(2, 0), I_BBcm_B(2, 1)));

    // Add a rigid body to model each link.
    plant->AddRigidBody(link->Name(), M_BBo_B);
  }

  // Add all the joints
  for (uint64_t joint_index = 0; joint_index < model->JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint* joint = model->JointByIndex(joint_index);
    const sdf::JointAxis* axis = joint->Axis();

    // Get the location of the joint in the model frame.
    const Isometry3d X_MJ = ToIsometry3(joint->Pose());

    // Get the location of the child link in the model frame.
    const Isometry3d X_MC = ToIsometry3(
        model->LinkByName(joint->ChildLinkName())->Pose());

    // Compute the location of the child joint in the child link's frame.
    const Isometry3d X_CJ = X_MC.inverse() * X_MJ;

    // Only supporting revolute joints for now.
    if (joint->Type() == sdf::JointType::REVOLUTE) {
      // TODO(nkoenig) Check that the links exist before creating the
      /// joint.

      // Special case for a joint connected to the world.
      if (joint->ParentLinkName().empty() ||
          joint->ParentLinkName() == "world") {
        plant->AddJoint<RevoluteJoint>(joint->Name(),
            plant->world_body(), {},
            plant->GetBodyByName(joint->ChildLinkName()), X_CJ,
            ToVector3(axis->Xyz()));
      } else {
        // Get the location of the parent link in the model frame.
        const Isometry3d X_MP = ToIsometry3(
            model->LinkByName(joint->ParentLinkName())->Pose());

        // Compute the location of the parent joint in the parent link's frame.
        const Isometry3d X_PJ = X_MP.inverse() * X_MJ;

        plant->AddJoint<RevoluteJoint>(joint->Name(),
            plant->GetBodyByName(joint->ParentLinkName()), X_PJ,
            plant->GetBodyByName(joint->ChildLinkName()), X_CJ,
            ToVector3(axis->Xyz()));
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
