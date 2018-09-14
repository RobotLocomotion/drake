#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

#include <memory>
#include <utility>
#include <vector>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/joints/weld_joint.h"
#include "drake/multibody/multibody_tree/parsing/parser_path_utils.h"
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
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::detail::ToIsometry3;
using drake::multibody::parsing::detail::ToVector3;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using drake::multibody::WeldJoint;
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
  const ignition::math::Matrix3d I = inertial.MassMatrix().Moi();
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
    ModelInstanceIndex model_instance, const MultibodyPlant<double>& plant) {
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
    return plant.GetBodyByName(link_name, model_instance);
  }
}

// Extracts a Vector3d representation of the joint axis for joints with an axis.
Vector3d ExtractJointAxis(const sdf::Model& model_spec,
                          const sdf::Joint& joint_spec) {
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
    // Pose of the joint frame J in the frame of the child link C.
    const Isometry3d X_CJ = ToIsometry3(joint_spec.Pose());
    // Get the pose of the child link C in the model frame M.
    const Isometry3d X_MC =
        ToIsometry3(model_spec.LinkByName(joint_spec.ChildLinkName())->Pose());
    const Isometry3d X_MJ = X_MC * X_CJ;
    // axis_J actually contains axis_M, expressed in the model frame M.
    axis_J = X_MJ.linear().transpose() * axis_J;
  }
  return axis_J;
}

// Helper to parse the damping for a given joint specification.
// Right now we only parse the <damping> tag.
// An exception is thrown if the provided damping value is negative or if there
// is no <axis> under <joint>.
double ParseJointDamping(const sdf::Joint& joint_spec) {
  DRAKE_DEMAND(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }
  const double damping = axis->Damping();
  if (damping < 0) {
    throw std::runtime_error(
        "Joint damping is negative for joint '" + joint_spec.Name() + "'. "
            "Joint damping must be a non-negative number.");
  }
  return damping;
}

// Extracts the effort limit from a joint specification and adds an actuator if
// the value is non-zero. In SDF, effort limits are specified in
// <joint><axis><limit><effort>. In Drake, we understand that joints with an
// effort limit of zero are not actuated.
// Only available for "revolute" and "prismatic" joints.
void AddJointActuatorFromSpecification(
    const sdf::Joint &joint_spec, const Joint<double>& joint,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);

  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }

  double max_effort = axis->Effort();

  // The SDF specification defines this max_effort = -1 when no limit is
  // provided (a non-zero value). In Drake we interpret a value of exactly zero
  // as a way to specify un-actuated joints. Thus, the user would say
  // <effort>0</effort> for un-actuated joints.
  if (max_effort != 0) {
    // TODO(amcastro-tri): For positive max_effort values, store it and use it
    // to limit input actuation.
    plant->AddJointActuator(joint_spec.Name(), joint);
  }
}

// Returns joint limits as the pair (lower_limit, upper_limit).
// The units of the limits depend on the particular joint type. Units are meters
// for prismatic joints and radians for revolute joints.
// This method throws an exception if the joint type is not one of revolute or
// prismatic.
std::pair<double, double> ParseJointLimits(const sdf::Joint& joint_spec) {
  DRAKE_THROW_UNLESS(joint_spec.Type() == sdf::JointType::REVOLUTE ||
      joint_spec.Type() == sdf::JointType::PRISMATIC);
  // Axis specification.
  const sdf::JointAxis* axis = joint_spec.Axis();
  if (axis == nullptr) {
    throw std::runtime_error(
        "An axis must be specified for joint '" + joint_spec.Name() + "'");
  }
  const double lower_limit = axis->Lower();
  const double upper_limit = axis->Upper();
  if (lower_limit > upper_limit) {
    throw std::runtime_error(
        "The lower limit must be lower (or equal) than the upper limit for "
        "joint '" + joint_spec.Name() + "'.");
  }
  return std::make_pair(axis->Lower(), axis->Upper());
}

// Helper method to add joints to a MultibodyPlant given an sdf::Joint
// specification object.
void AddJointFromSpecification(
    const sdf::Model& model_spec, const sdf::Joint& joint_spec,
    ModelInstanceIndex model_instance, MultibodyPlant<double>* plant) {
  // Pose of the model frame M in the world frame W.
  const Isometry3d X_WM = ToIsometry3(model_spec.Pose());

  const Body<double>& parent_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ParentLinkName(), model_instance, *plant);
  const Body<double>& child_body = GetBodyByLinkSpecificationName(
      model_spec, joint_spec.ChildLinkName(), model_instance, *plant);

  // Get the pose of frame J in the frame of the child link C, as specified in
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
  // will ALWAYS return X_CJ.
  const Isometry3d X_CJ = ToIsometry3(joint_spec.Pose());

  // Get the pose of the child link C in the model frame M.
  const Isometry3d X_MC =
      ToIsometry3(model_spec.LinkByName(joint_spec.ChildLinkName())->Pose());

  // Pose of the joint frame J in the model frame M.
  const Isometry3d X_MJ = X_MC * X_CJ;

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

  switch (joint_spec.Type()) {
    case sdf::JointType::FIXED: {
      plant->AddJoint<WeldJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ,
          Isometry3d::Identity() /* X_JpJc */);
      break;
    }
    case sdf::JointType::PRISMATIC: {
      const double damping = ParseJointDamping(joint_spec);
      Vector3d axis_J = ExtractJointAxis(model_spec, joint_spec);
      const std::pair<double, double> limits = ParseJointLimits(joint_spec);
      const auto& joint = plant->AddJoint<PrismaticJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ, axis_J, limits.first, limits.second, damping);
      AddJointActuatorFromSpecification(joint_spec, joint, plant);
      break;
    }
    case sdf::JointType::REVOLUTE: {
      const double damping = ParseJointDamping(joint_spec);
      Vector3d axis_J = ExtractJointAxis(model_spec, joint_spec);
      const std::pair<double, double> limits = ParseJointLimits(joint_spec);
      const auto& joint = plant->AddJoint<RevoluteJoint>(
          joint_spec.Name(),
          parent_body, X_PJ,
          child_body, X_CJ, axis_J, limits.first, limits.second, damping);
      AddJointActuatorFromSpecification(joint_spec, joint, plant);
      break;
    }
    default: {
      throw std::logic_error(
          "Joint type not supported for joint '" + joint_spec.Name() + "'.");
    }
  }
}

// Helper method to load an SDF file and read the contents into an sdf::Root
// object.
std::string LoadSdf(
    sdf::Root* root,
    parsing::PackageMap* package_map,
    const std::string& file_name) {

  const std::string full_path = parsing::GetFullPath(file_name);

  // Load the SDF file.
  sdf::Errors errors = root->Load(full_path);

  // Check for any errors.
  if (!errors.empty()) {
    std::string error_accumulation("From AddModelFromSdfFile():\n");
    for (const auto& e : errors)
      error_accumulation += "Error: " + e.Message() + "\n";
    throw std::runtime_error(error_accumulation);
  }

  // TODO(sam.creasey) Add support for using an existing package map.
  package_map->PopulateUpstreamToDrake(full_path);

  // Uses the directory holding the SDF to be the root directory
  // in which to search for files referenced within the SDF file.
  std::string root_dir = ".";
  size_t found = full_path.find_last_of("/\\");
  if (found != std::string::npos) {
    root_dir = full_path.substr(0, found);
  }

  return root_dir;
}

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
void AddLinksFromSpecification(
    const ModelInstanceIndex model_instance,
    const sdf::Model& model,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph,
    const parsing::PackageMap& package_map,
    const std::string& root_dir) {

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
    const RigidBody<double>& body =
        plant->AddRigidBody(link.Name(), model_instance, M_BBo_B);

    if (scene_graph != nullptr) {
      for (uint64_t visual_index = 0; visual_index < link.VisualCount();
           ++visual_index) {
        const sdf::Visual sdf_visual = detail::ResolveVisualUri(
            *link.VisualByIndex(visual_index), package_map, root_dir);
        unique_ptr<GeometryInstance> geometry_instance =
            detail::MakeGeometryInstanceFromSdfVisual(sdf_visual);
        // We check for nullptr in case someone decided to specify an SDF
        // <empty/> geometry.
        if (geometry_instance) {
          plant->RegisterVisualGeometry(
              body, geometry_instance->pose(), geometry_instance->shape(),
              geometry_instance->name(), geometry_instance->visual_material(),
              scene_graph);
        }
      }

      for (uint64_t collision_index = 0;
           collision_index < link.CollisionCount(); ++collision_index) {
        const sdf::Collision& sdf_collision =
            *link.CollisionByIndex(collision_index);
        const sdf::Geometry& sdf_geometry = *sdf_collision.Geom();
        if (sdf_geometry.Type() != sdf::GeometryType::EMPTY) {
          const Isometry3d X_LG =
              detail::MakeGeometryPoseFromSdfCollision(sdf_collision);
          std::unique_ptr<geometry::Shape> shape =
              detail::MakeShapeFromSdfGeometry(sdf_geometry);
          const CoulombFriction<double> coulomb_friction =
              detail::MakeCoulombFrictionFromSdfCollisionOde(sdf_collision);
          plant->RegisterCollisionGeometry(body, X_LG, *shape,
                                           sdf_collision.Name(),
                                           coulomb_friction, scene_graph);
        }
      }
    }
  }
}

// Helper method to add a model to a MultibodyPlant given an sdf::Model
// specification object.
ModelInstanceIndex AddModelFromSpecification(
    const sdf::Model& model,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph,
    const parsing::PackageMap& package_map,
    const std::string& root_dir) {

  const ModelInstanceIndex model_instance =
    plant->AddModelInstance(model_name);

  AddLinksFromSpecification(
      model_instance, model, plant, scene_graph, package_map, root_dir);

  // Add all the joints
  for (uint64_t joint_index = 0; joint_index < model.JointCount();
       ++joint_index) {
    // Get a pointer to the SDF joint, and the joint axis information.
    const sdf::Joint& joint = *model.JointByIndex(joint_index);
    AddJointFromSpecification(model, joint, model_instance, plant);
  }

  return model_instance;
}
}  // namespace

ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    const std::string& model_name_in,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  sdf::Root root;
  parsing::PackageMap package_map;

  std::string root_dir = LoadSdf(&root, &package_map, file_name);

  if (root.ModelCount() != 1) {
    throw std::runtime_error("File must have a single <model> element.");
  }

  // Get the only model in the file.
  const sdf::Model& model = *root.ModelByIndex(0);

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  const std::string model_name =
      model_name_in.empty() ? model.Name() : model_name_in;

  return AddModelFromSpecification(
      model, model_name, plant, scene_graph, package_map, root_dir);
}

ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  return AddModelFromSdfFile(file_name, "", plant, scene_graph);
}

std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(!plant->is_finalized());

  sdf::Root root;
  parsing::PackageMap package_map;

  std::string root_dir = LoadSdf(&root, &package_map, file_name);

  // Throw an error if there are no models or worlds.
  if (root.ModelCount() == 0 && root.WorldCount() == 0) {
    throw std::runtime_error(
        "File must have at least one <model>, or <world> with one "
        "child <model> element.");
  }

  // Only one world in an SDF file is allowed.
  if (root.WorldCount() > 1) {
    throw std::runtime_error("File must contain only one <world>.");
  }

  // Do not allow a <world> to have a <model> sibling.
  if (root.ModelCount() > 0 && root.WorldCount() > 0) {
    throw std::runtime_error("A <world> and <model> cannot be siblings.");
  }

  if (scene_graph != nullptr && !plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  std::vector<ModelInstanceIndex> model_instances;

  // At this point there should be only Models or a single World at the Root
  // levelt.
  if (root.ModelCount() > 0) {
    // Load all the models at the root level.
    for (uint64_t i = 0; i < root.ModelCount(); ++i) {
      // Get the model.
      const sdf::Model& model = *root.ModelByIndex(i);
      model_instances.push_back(AddModelFromSpecification(
            model, model.Name(), plant, scene_graph, package_map, root_dir));
    }
  } else {
    // Load the world and all the models in the world.
    const sdf::World& world = *root.WorldByIndex(0);

    for (uint64_t model_index = 0; model_index < world.ModelCount();
        ++model_index) {
      // Get the model.
      const sdf::Model& model = *world.ModelByIndex(model_index);
      model_instances.push_back(AddModelFromSpecification(
            model, model.Name(), plant, scene_graph, package_map, root_dir));
    }
  }

  return model_instances;
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
