#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

#include <memory>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/parsing/scene_graph_parser_detail.h"
#include "drake/multibody/multibody_tree/parsing/sdf_parser_common.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace parsing {

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using drake::geometry::GeometryInstance;
using drake::geometry::SceneGraph;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::detail::ToIsometry3;
using drake::multibody::parsing::detail::ToVector3;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using std::unique_ptr;

// Unnamed namespace for free functions local to this file.
namespace {
// Given an ignition::math::Inertial object, extract a RotationalInertia object
// for the rotational inertia of body B, about its center of mass Bcm and,
// expressed in the inertial frame Bi (as specified in <inertial> in the SDF
// file.)
RotationalInertia<double> ExtractRotationalInertiaAboutBcmExpressedInBi(
    const ignition::math::Inertiald &inertial) {
  // TODO(amcastro-tri): Verify that ignition::math::Inertial::MOI() ALWAYS is
  // expresed in the body frame B, regardless of how a user might have
  // specified frames in the sdf file. That is, that it always returns R_BBcm_B.
  // TODO(amcastro-tri): Verify that ignition::math::Inertial::MassMatrix()
  // ALWAYS is in the inertial frame Bi, regardless of how a user might have
  // specified frames in the sdf file. That is, that it always returns
  // M_BBcm_Bi.
  const ignition::math::Matrix3d I = inertial.MassMatrix().MOI();
  return RotationalInertia<double>(I(0, 0), I(1, 1), I(2, 2),
                                   I(1, 0), I(2, 0), I(2, 1));
}

// Helper method to extract the SpatialInertia M_BBo_B of body B, about its body
// frame origin Bo and, expressed in body frame B, from an ignition::Inertial
// object.
SpatialInertia<double> ExtractSpatialInertiaAboutBoExpressedInB(
    const ignition::math::Inertiald& Inertial_BBcm_Bi) {
  double mass = Inertial_BBcm_Bi.MassMatrix().Mass();

  const RotationalInertia<double> I_BBcm_Bi =
      ExtractRotationalInertiaAboutBcmExpressedInBi(Inertial_BBcm_Bi);

  // Pose of the "<inertial>" frame Bi in the body frame B.
  // TODO(amcastro-tri): Verify we don't get funny results when X_BBi is not
  // the identity matrix.
  // TODO(amcastro-tri): SDF seems to provide <inertial><frame/></inertial>
  // together with <inertial><pose/></inertial>. It'd seem then the frame B
  // in X_BI could be another frame. We rely on Inertial::Pose() to ALWAYS
  // give us X_BI. Verify this.
  const Isometry3d X_BBi = ToIsometry3(Inertial_BBcm_Bi.Pose());

  // B and Bi are not necessarily aligned.
  const Matrix3d R_BBi = X_BBi.linear();

  // Re-express in frame B as needed.
  const RotationalInertia<double> I_BBcm_B = I_BBcm_Bi.ReExpress(R_BBi);

  // Bi's origin is at the COM as documented in
  // http://sdformat.org/spec?ver=1.6&elem=link#inertial_pose
  const Vector3d p_BoBcm_B = X_BBi.translation();

  // Return the spatial inertia M_BBo_B of body B, about its body frame origin
  // Bo, and expressed in the body frame B.
  return SpatialInertia<double>::MakeFromCentralInertia(
      mass, p_BoBcm_B, I_BBcm_B);
}

// Helper method to retrieve a Body given the name of the link specification.
const Body<double>& GetBodyByLinkSpecificationName(
    const sdf::Model& model, const std::string& link_name,
    const MultibodyPlant<double>& plant) {
  // SDF's convention to indicate a joint is connected to the world is to either
  // name the corresponding link "world" or just leave it unnamed.
  // Thus this the "if" statement in the following line.
  if (link_name.empty() || link_name == "world") {
    return plant.world_body();
  } else {
    // TODO(amcastro-tri): Remove this when sdformat guarantees a model
    // with basic structural checks.
    if (!model.LinkNameExists(link_name)) {
      throw std::logic_error("There is no parent link named '" +
          link_name + "' in the model.");
    }
    return plant.GetBodyByName(link_name);
  }
}

// Extracts a Vector3d representation of the joint axis for joints with an axis.
Vector3d ExtractJointAxis(const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  // Joint axis, by default in the joint frame J.
  // TODO(amcastro-tri): Verify JointAxis::UseParentModelFrame is actually
  // supported by sdformat.
  Vector3d axis_J = ToVector3(axis->Xyz());
  if (axis->UseParentModelFrame()) {
    const Isometry3d X_MJ = ToIsometry3(joint_spec.Pose());
    // axis_J actually contains axis_M, expressed in the model frame M.
    axis_J = X_MJ.linear().transpose() * axis_J;
  }
  return axis_J;
}

// Helper method to add joints to a MultibodyPlant given an sdf::Joint
// specification object.
void AddJointFromSpecification(
    const sdf::Model& model_spec, const sdf::Joint& joint_spec,
    MultibodyPlant<double>* plant) {
  // Pose of the model frame M in the world frame W.
  const Isometry3d X_WM = ToIsometry3(model_spec.Pose());

  const Body<double>& parent_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ParentLinkName(), *plant);
  const Body<double>& child_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ChildLinkName(), *plant);

  // Get the pose of frame J in the model frame M, as specified in
  // <joint> <pose> ... </pose></joint>.
  // TODO(amcastro-tri): Verify sdformat supports frame specifications
  // correctly.
  // There are many ways by which a joint frame pose can be specified in SDF:
  //  - <joint> <pose> </pose></joint>.
  //  - <joint> <pose> <frame/> </pose></joint>.
  //  - <joint> <frame> <pose> <frame/> </pose> </frame> </joint>.
  // And combinations of the above?
  // There is no way to verify at this level which one is supported or not.
  // Here we trust that no mather how a user specified the file, joint.Pose()
  // will ALWAYS return X_MJ.
  const Isometry3d X_MJ = ToIsometry3(joint_spec.Pose());

  // Get the pose of the child link C in the model frame M.
  const Isometry3d X_MC =
      ToIsometry3(model_spec.LinkByName(joint_spec.ChildLinkName())->Pose());

  // Compute the location of the joint in the child link's frame.
  const Isometry3d X_CJ = X_MC.inverse() * X_MJ;

  // Pose of the frame J in the parent body frame P.
  optional<Isometry3d> X_PJ;
  // We need to treat the world case separately since sdformat does not create
  // a "world" link from which we can request its pose (which in that case would
  // be the identity).
  if (parent_body.index() == world_index()) {
    X_PJ = X_WM * X_MJ;  // Since P == W.
  } else {
    // Get the pose of the parent link P in the model frame M.
    const Isometry3d X_MP =
        ToIsometry3(model_spec.LinkByName(joint_spec.ParentLinkName())->Pose());
    X_PJ = X_MP.inverse() * X_MJ;
  }

  // If P and J are coincident, we won't create a new frame for J, but use frame
  // P directly. We indicate that by passing a nullopt.
  if (X_PJ.value().isApprox(Isometry3d::Identity())) X_PJ = nullopt;

  // Only supporting revolute joints for now.
  switch (joint_spec.Type()) {
    case sdf::JointType::REVOLUTE: {
      Vector3d axis_J = ExtractJointAxis(joint_spec);
      plant->AddJoint<RevoluteJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ, axis_J);
      break;
    }
    case sdf::JointType::PRISMATIC: {
      Vector3d axis_J = ExtractJointAxis(joint_spec);
      plant->AddJoint<PrismaticJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ, axis_J);
      break;
    }
    default: {
      throw std::logic_error(
          "Joint type not supported for joint '" + joint_spec.Name() + "'.");
    }
  }
}

}  // namespace

void AddModelFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
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
    throw std::runtime_error("File must have a single <model> element.");
  }

  // Get the only model in the file.
  const sdf::Model& model = *root.ModelByIndex(0);

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  // Add all the links
  for (uint64_t link_index = 0; link_index < model.LinkCount(); ++link_index) {
    const sdf::Link& link = *model.LinkByIndex(link_index);

    // Get the link's inertia relative to the Bcm frame.
    // sdf::Link::Inertial() provides a representation for the SpatialInertia
    // M_Bcm_Bi of body B, about its center of mass Bcm, and expressed in an
    // inertial frame Bi as defined in <inertial> <pose></pose> </inertial>.
    // Per SDF specification, Bi's origin is at the COM Bcm, but Bi is not
    // necessarily aligned with B.
    const ignition::math::Inertiald& Inertial_Bcm_Bi = link.Inertial();

    const SpatialInertia<double> M_BBo_B =
        ExtractSpatialInertiaAboutBoExpressedInB(Inertial_Bcm_Bi);

    // Add a rigid body to model each link.
    const RigidBody<double>& body = plant->AddRigidBody(link.Name(), M_BBo_B);

    if (scene_graph != nullptr) {
      for (uint64_t visual_index = 0; visual_index < link.VisualCount();
           ++visual_index) {
        const sdf::Visual& sdf_visual = *link.VisualByIndex(visual_index);
        unique_ptr<GeometryInstance> geometry_instance =
            detail::MakeGeometryInstanceFromSdfVisual(sdf_visual);
        // We check for nullptr in case someone decided to specify an SDF
        // <empty/> geometry.
        if (geometry_instance) {
          plant->RegisterVisualGeometry(
              body, geometry_instance->pose(), geometry_instance->shape(),
              scene_graph);
        }
      }
    }
  }

  // Add all the joints
  for (uint64_t joint_index = 0; joint_index < model.JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint& joint = *model.JointByIndex(joint_index);
    AddJointFromSpecification(model, joint, plant);
  }
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
