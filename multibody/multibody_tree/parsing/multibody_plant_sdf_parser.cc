#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

#include <memory>

#include <sdf/sdf.hh>

#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace parsing {

using Eigen::Matrix3d;
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
// a Vector3<double> instance.
Vector3<double> ToVector3(const ignition::math::Vector3d& vector) {
  return Vector3<double>(vector.X(), vector.Y(), vector.Z());
}

// Helper function to express an ignition::math::Pose3d instance as
// an Isometry3<double> instance.
Isometry3<double> ToIsometry3(const ignition::math::Pose3d& pose) {
  const Isometry3<double>::TranslationType translation(ToVector3(pose.Pos()));
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return translation * rotation;
}

RotationalInertia<double> ExtractRotationalInertia(
    const ignition::math::Inertiald& inertial) {
  // TODO(amcastro-tri): From MOI() method docs: "Get the moment of inertia
  // matrix expressed in the base coordinate frame.", What is the "base frame"?
  const ignition::math::Matrix3d I = inertial.MOI();
  return RotationalInertia<double>(I(0, 0), I(1, 1), I(2, 2),
                                   I(1, 0), I(2, 0), I(2, 1));
}

SpatialInertia<double> ExtractSpatialInertia(
    const ignition::math::Inertiald& Inertial_BBcm_Bi) {
  double mass = Inertial_BBcm_Bi.MassMatrix().Mass();

  const RotationalInertia<double> I_BBcm_Bi =
      ExtractRotationalInertia(Inertial_BBcm_Bi);

  // Pose of the "<inertial>" frame Bi in the body frame B.
  // TODO(amcastro-tri): Verify we don't get funny results when X_BBi is not
  // the identity matrix.
  // TODO(amcastro-tri): SDF seems to provide <inertial><frame/></inertial>
  // together with <inertial><pose/></inertial>. It'd seem then the frame B
  // in X_BI could be another frame. This is currently NOT supported by this
  // parser.
  const Isometry3d X_BBi = ToIsometry3(Inertial_BBcm_Bi.Pose());

  // B and Bi are not necessarily aligned.
  const Matrix3d R_BBi = X_BBi.linear();

  // Re-express in frame B as needed.
  const RotationalInertia<double> I_BBcm_B = I_BBcm_Bi.ReExpress(R_BBi);

  // Bi's origin is at the COM.
  const Vector3d p_BoBcm_B = X_BBi.translation();

  // Return the spatial inertia M_BBo_B of body B, about its body frame origin
  // Bo, and expressed in the body frame B.
  return SpatialInertia<double>::MakeFromCentralInertia(
      mass, p_BoBcm_B, I_BBcm_B);
}

}  // namespace

void AddModelFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  // Load the SDF string
  sdf::Root root;
  sdf::Errors errors = root.Load(file_name);

  // Check for any errors.
  if (!errors.empty()) {
    std::string error_accumulation("From AddModelFromSdfString():\n");
    for (const auto& e : errors)
      error_accumulation += "Error: " + e.Message() + "\n";
    throw std::runtime_error(error_accumulation);
  }

  if (root.ModelCount() != 1) {
    throw std::logic_error("File must have a single <model> element.");
  }

  // Get a pointer to the only model in the file.
  const sdf::Model* model = root.ModelByIndex(0);
  DRAKE_ASSERT(model != nullptr);

  // Add all the links
  for (uint64_t link_index = 0; link_index < model->LinkCount(); ++link_index) {
    const sdf::Link* link = model->LinkByIndex(link_index);
    DRAKE_ASSERT(link != nullptr);

    // Get the link's inertia relative to the Bcm frame.
    // Inertial provides a representation for the SpatialInertia M_Bcm_Bi of
    // body B, about its center of mass Bcm, and expressed in an inertial frame
    // Bi as defined in <inertial> <pose></pose> </inertial>.
    // Per SDF specification, Bi's origin is at the COM Bcm, but Bi is not
    // necessarily aligned with B.
    const ignition::math::Inertiald& Inertial_Bcm_Bi = link->Inertial();

    const SpatialInertia<double> M_BBo_B =
        ExtractSpatialInertia(Inertial_Bcm_Bi);

    // Add a rigid body to model each link.
    plant->AddRigidBody(link->Name(), M_BBo_B);
  }

  // Pose of the model frame M in the world frame W.
  const Isometry3d X_WM = ToIsometry3(model->Pose());

  // Add all the joints
  for (uint64_t joint_index = 0; joint_index < model->JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint* joint = model->JointByIndex(joint_index);

    // Axis expressed in the joint frame J.
    const sdf::JointAxis* axis = joint->Axis();

    // Get the pose of frame J in the model frame M, as specified in
    // <joint> <pose> ... </pose></joint>.
    // TODO(amcastro-tri): Verify sdformat supports frame specifications
    // correctly.
    // There are many ways by which a joint frame pose can be specified in SDF:
    //  - <joint> <pose> </pose></joint>.
    //  - <joint> <pose> <frame/> </pose></joint>.
    //  - <joint> <frame><pose> <frame/> </pose></frame> </pose></joint>.
    // And combinations of the above?
    // There is no way to verify at this level which one is supported or not.
    // Here we trust that no mather how a user specified the file, joint->Pose()
    // will ALWAYS return X_MJ.
    const Isometry3d X_MJ = ToIsometry3(joint->Pose());

    // Get the pose of the child link C in the model frame M.
    const Isometry3d X_MC =
        ToIsometry3(model->LinkByName(joint->ChildLinkName())->Pose());

    // Compute the location of the child joint in the child link's frame.
    const Isometry3d X_CJ = X_MC.inverse() * X_MJ;

    // Only supporting revolute joints for now.
    switch (joint->Type()) {
      case sdf::JointType::REVOLUTE: {
        // Joint axis, by default in the joint frame J.
        // TODO(amcastro-tri): Verify this capability indeed is supported by
        // sdformat.
        Vector3d axis_J = ToVector3(axis->Xyz());
        if (axis->UseParentModelFrame()) {
          // axis_J actually contains axis_M, expressed in the model frame M.
          axis_J = X_MJ.inverse() * axis_J;
        }

        // Special case for a joint connected to the world.
        if (joint->ParentLinkName().empty() ||
            joint->ParentLinkName() == "world") {
          // TODO(amcastro-tri): Remove these when sdformat guarantees a model
          // with basic structural checks.
          if (!model->LinkNameExists(joint->ChildLinkName())) {
            throw std::logic_error("There is no child link named '" +
                joint->ChildLinkName() + "' in the model.");
          }

          // If X_PG is left empty, P IS the world frame W, no new frame gets
          // created.
          optional<Isometry3d> X_PJ;
          if (!X_WM.isApprox(Isometry3d::Identity())) X_PJ = X_WM;

          plant->AddJoint<RevoluteJoint>(
              joint->Name(),
              plant->world_body(), X_PJ,
              plant->GetBodyByName(joint->ChildLinkName()), X_CJ, axis_J);
        } else {
          // TODO(amcastro-tri): Remove these when sdformat guarantees a model
          // with basic structural checks.
          if (!model->LinkNameExists(joint->ParentLinkName())) {
            throw std::logic_error("There is no parent link named '" +
                joint->ParentLinkName() + "' in the model.");
          }
          if (!model->LinkNameExists(joint->ChildLinkName())) {
            throw std::logic_error("There is no child link named '" +
                joint->ChildLinkName() + "' in the model.");
          }

          // Get the pose of the parent link P in the model frame M.
          const Isometry3d X_MP =
              ToIsometry3(model->LinkByName(joint->ParentLinkName())->Pose());

          // Pose of the frame J in the parent body frame P.
          const Isometry3d X_PJ = X_MP.inverse() * X_MJ;

          plant->AddJoint<RevoluteJoint>(
              joint->Name(),
              plant->GetBodyByName(joint->ParentLinkName()), X_PJ,
              plant->GetBodyByName(joint->ChildLinkName()), X_CJ,
              axis_J);
        }
        break;
      }
      default: {
        throw std::logic_error(
            "Joint type not supported for joint '" + joint->Name() + "'.");
      }
    }  // switch
  }  // joint_index
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
