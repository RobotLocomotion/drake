#include "drake/multibody/multibody_tree/parsing/sdf/sdf_parser.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "sdf/sdf.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

// Helper function to express an ignition::math::Vector3d instance as
// a Vector3<double> instance.
Vector3<double> ToVector(const ignition::math::Vector3d& vector) {
  return Vector3<double>(vector.X(), vector.Y(), vector.Z());
}

// Helper function to express an ignition::math::Pose3d instance as
// an Isometry3<double> instance.
Isometry3<double> ToIsometry(const ignition::math::Pose3d& pose) {
  const Isometry3<double>::TranslationType translation(ToVector(pose.Pos()));
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return translation * rotation;
}

// Parses a pose from the given SDF element.
Isometry3<double> ParsePose(sdf::ElementPtr sdf_pose_element) {
  DRAKE_DEMAND(sdf_pose_element != nullptr);
  DRAKE_DEMAND(sdf_pose_element->GetName() == "pose");
  return ToIsometry(sdf_pose_element->Get<ignition::math::Pose3d>());
}

// Parses inertial information (mass, COM) from the given SDF
// element and updates the given body accordingly.
void SDFParser::ParseInertial(sdf::ElementPtr sdf_inertial_element,
                              SDFLink* link) {
  DRAKE_DEMAND(sdf_inertial_element != nullptr);
  DRAKE_DEMAND(sdf_inertial_element->GetName() == "inertial");
  DRAKE_DEMAND(link != nullptr);

  // Inertial frame's (I) pose in link's frame (L).
  auto X_LI = Isometry3<double>::Identity();
  if (sdf_inertial_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_inertial_element->GetElement("pose");
    X_LI = ParsePose(sdf_pose_element);
  }
  // So far this parsing assumes the pose was given in the link's frame L.
  // TODO: register inertial frame and set its pose in an arbitrary measured-in
  // frame.
  link->set_inertial_frame_pose(X_LI);

  if (sdf_inertial_element->HasElement("mass")) {
    sdf::ElementPtr sdf_mass_element = sdf_inertial_element->GetElement("mass");
    link->set_mass(sdf_mass_element->Get<double>());
  }

  // Parse the <inertia> element.
  if (sdf_inertial_element->HasElement("inertia")) {
    sdf::ElementPtr sdf_inertia_element =
        sdf_inertial_element->GetElement("inertia");
    // Per SDF specification, the following elements are required within the
    // <inertia> element.
    const double Ixx = sdf_inertia_element->GetElement("ixx")->Get<double>();
    const double Ixy = sdf_inertia_element->GetElement("ixy")->Get<double>();
    const double Ixz = sdf_inertia_element->GetElement("ixz")->Get<double>();
    const double Iyy = sdf_inertia_element->GetElement("iyy")->Get<double>();
    const double Iyz = sdf_inertia_element->GetElement("iyz")->Get<double>();
    const double Izz = sdf_inertia_element->GetElement("izz")->Get<double>();
    // Inertia matrix about Io (Lcm) and expressed in I.
    Matrix3<double> I_Io_I;
    I_Io_I(0, 0) = Ixx; I_Io_I(0, 1) = Ixy; I_Io_I(0, 2) = Ixz;
    I_Io_I(1, 0) = Ixy; I_Io_I(1, 1) = Iyy; I_Io_I(1, 2) = Iyz;
    I_Io_I(2, 0) = Ixz; I_Io_I(2, 1) = Iyz; I_Io_I(2, 2) = Izz;
    link->set_inertia_matrix(I_Io_I);
  }
}

// Parses inertial, visual and collision details of a body, if any, from the
// given SDF element and adds a RigidBody instance to the given tree as part
// of the given model instance. The frame cache is updated with the body frame's
// pose (B) in its model frame (D).
void SDFParser::ParseLink(const sdf::ElementPtr sdf_link_element,
                          SDFModel* sdf_model) {
  DRAKE_DEMAND(sdf_link_element != nullptr);
  DRAKE_DEMAND(sdf_link_element->GetName() == "link");
  DRAKE_DEMAND(sdf_model != nullptr);

  const auto link_name = sdf_link_element->Get<std::string>("name");
  SDFLink& sdf_link = sdf_model->AddLink(link_name);

  // Parse the pose of the link's frame (L) pose in the model frame (D).
  auto X_DL = Isometry3<double>::Identity();
  if (sdf_link_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_link_element->GetElement("pose");
    X_DL = ParsePose(sdf_pose_element);
  }
  sdf_link.set_pose_in_model(X_DL);
  // "Remember" the pose of this new link in the model frame D.
  GetModelFrameCache(sdf_model->name()).Update(
      sdf_model->name(), link_name, X_DL);

  if (sdf_link_element->HasElement("inertial")) {
    sdf::ElementPtr sdf_inertial_element =
        sdf_link_element->GetElement("inertial");
    ParseInertial(sdf_inertial_element, &sdf_link);
  }

  // TODO: parse visual and collision elements.
}

#if 0
// Parses a joint type from the given SDF element and instantiates the
// corresponding DrakeJoint instance. It is assumed that the parent body
// frame is already present in the frame cache.
std::unique_ptr<DrakeJoint>
ParseJointType(const sdf::ElementPtr sdf_joint_element,
               const ModelInstance& instance,
               const RigidBody<double>& parent_body,
               const FrameCache<double>& frame_cache) {
  DRAKE_DEMAND(sdf_joint_element != nullptr);
  DRAKE_DEMAND(sdf_joint_element->GetName() == "joint");
  const auto joint_name = sdf_joint_element->Get<std::string>("name");
  const auto joint_type = sdf_joint_element->Get<std::string>("type");
  if (joint_type == "revolute") {
    sdf::ElementPtr sdf_axis_element = sdf_joint_element->GetElement("axis");
    sdf::ElementPtr sdf_xyz_element = sdf_axis_element->GetElement("xyz");
    Vector3<double> axis_of_rotation = ToVector(
        sdf_xyz_element->Get<ignition::math::Vector3d>());
    if (sdf_joint_element->HasElement("use_parent_model_frame")) {
      sdf::ElementPtr sdf_use_parent_frame_element =
          sdf_joint_element->GetElement("use_parent_model_frame");
      if (sdf_use_parent_frame_element->Get<bool>()) {
        // Axis of rotation is defined in the model frame (D).

        // Joint frame's (M) pose in model frame (D).
        const Isometry3<double> X_DM =
            frame_cache.Transform(instance.name, joint_name);
        // Axis of rotation must be rotated back to the joint
        // frame (M), so the inverse of the rotational part of the
        // pose of the joint frame (M) in the model frame (D)
        // is applied.
        axis_of_rotation = X_DM.linear().inverse() * axis_of_rotation;
      }
    }
    // To instantiate the joint we require the joint location as seen from
    // the parent body frame. That is, the joint frame's (F) pose in the parent
    // body frame (P).
    const Isometry3<double> X_PF =
        frame_cache.Transform(parent_body.get_name(), joint_name);
    auto joint = std::make_unique<RevoluteJoint>(
        joint_name, X_PF, axis_of_rotation);
    return std::move(joint);
  }
  // TODO(hidmic): Support prismatic and fixed joints.
  DRAKE_ABORT_MSG("Unsupported joint type!");
}

// Parses a joint from the given SDF element and adds a DrakeJoint instance
// to link the parent and child bodies in the tree.
// It is assumed that all body frames are already present in the frame cache,
// which is further updated with the joint frame's (M) pose in the child body
// frame (B).
void ParseJoint(sdf::ElementPtr sdf_joint_element,
                const ModelInstance& instance,
                RigidBodyTree<double>* tree,
                FrameCache<double>* frame_cache) {
  DRAKE_DEMAND(sdf_joint_element != nullptr);
  DRAKE_DEMAND(sdf_joint_element->GetName() == "joint");
  DRAKE_DEMAND(tree != nullptr);
  DRAKE_DEMAND(frame_cache != nullptr);

  const auto joint_name = sdf_joint_element->Get<std::string>("name");
  sdf::ElementPtr sdf_parent_link_element =
      sdf_joint_element->GetElement("parent");
  const auto parent_link_name = sdf_parent_link_element->Get<std::string>();
  RigidBody<double>* parent_body = tree->FindBody(
      parent_link_name, instance.name, instance.id);

  sdf::ElementPtr sdf_child_link_element =
      sdf_joint_element->GetElement("child");
  const auto child_link_name = sdf_child_link_element->Get<std::string>();
  RigidBody<double>* child_body = tree->FindBody(
      child_link_name, instance.name, instance.id);

  // Joint frame's (M) pose in child body frame (B).
  auto X_BM = Isometry3<double>::Identity();
  if (sdf_joint_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_joint_element->GetElement("pose");
    // Joint poses specified in SDF files are, by default, in the child body
    // frame (B). See http://sdformat.org/spec?ver=1.4&elem=joint for details.
    X_BM = ParsePose(sdf_pose_element);
  }
  frame_cache->Update(child_link_name, joint_name, X_BM);

  // All child link's inertia, visual, and collision
  // elements reference frames must be this joint's frame.
  child_body->ApplyTransformToJointFrame(X_BM.inverse());

  tree->transformCollisionFrame(child_body, X_BM.inverse());

  // Update child link's parent and joint.
  child_body->setJoint(ParseJointType(
      sdf_joint_element, instance, *parent_body, *frame_cache));
  child_body->set_parent(parent_body);
}

// Welds all bodies that have no parent to the world for the given
// model instance in the given tree.
void FixDanglingLinks(const ModelInstance& instance,
                      RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree != nullptr);
  const int& world_index = RigidBodyTreeConstants::kWorldBodyIndex;
  const std::string world_name = RigidBodyTreeConstants::kWorldName;
  RigidBody<double>* world = tree->bodies[world_index].get();

  for (auto& body : tree->bodies) {
    // Filter out by model instance id but also make
    // sure we're not dealing with the world body.
    if (body->get_model_instance_id() == instance.id &&
        !body->has_joint() && body->get_name() != world_name) {
      // Fix dangling link body to the world body.
      auto fixed_joint = std::make_unique<FixedJoint>(
          body->get_name() + "_to_world_joint",
          Isometry3<double>::Identity());
      body->setJoint(std::move(fixed_joint));
      body->set_parent(world);
    }
  }
}
#endif

std::unique_ptr<SDFSpec> SDFParser::ParseSDFModelFromFile(
    const std::string& sdf_path) {
  sdf::SDFPtr parsed_sdf(new sdf::SDF());
  sdf::init(parsed_sdf);
  sdf::readFile(sdf_path, parsed_sdf);
  sdf::ElementPtr sdf_element = parsed_sdf->Root();
  DRAKE_DEMAND(sdf_element != nullptr);

  // version is a required attribute of <sdf>.
  DRAKE_DEMAND(sdf_element->HasAttribute("version"));
  std::string version = sdf_element->Get<std::string>("version");

  // Search for the first element that has a <model>.
  while (!sdf_element->HasElement("model")) {
    // To deal with both world and model SDFs, get next first
    // child element and check for model element existence.
    sdf_element = sdf_element->GetFirstElement();
    DRAKE_DEMAND(sdf_element != nullptr);
  }
  // This is the first <model> element found. Other <model> elements are
  // ignored.
  sdf::ElementPtr sdf_model_element = sdf_element->GetElement("model");

  auto spec = std::make_unique<SDFSpec>(version);
  ParseModel(sdf_model_element, spec.get());
  return spec;
}

// Parses all bodies and joints of a model from the given SDF element
// and adds RigidBody and DrakeJoint instances to describe such model
// within the given tree.
//
// Returns the model instance id.
void SDFParser::ParseModel(sdf::ElementPtr sdf_model_element, SDFSpec* spec) {
  DRAKE_DEMAND(sdf_model_element != nullptr);
  DRAKE_DEMAND(sdf_model_element->GetName() == "model");
  DRAKE_DEMAND(spec != nullptr);

  // Create a new SDF model.
  const auto model_name = sdf_model_element->Get<std::string>("name");
  SDFModel& sdf_model = spec->AddModel(model_name);
  model_name_to_frame_cache_map_.emplace(
      model_name, std::make_unique<FrameCache<double>>(model_name));

  if (sdf_model_element->HasElement("link")) {
    sdf::ElementPtr sdf_link_element = sdf_model_element->GetElement("link");
    while (sdf_link_element != nullptr) {
      ParseLink(sdf_link_element, &sdf_model);
      sdf_link_element = sdf_link_element->GetNextElement("link");
    }
  }

#if 0
  if (sdf_model_element->HasElement("joint")) {
    sdf::ElementPtr sdf_joint_element = sdf_model_element->GetElement("joint");
    while (sdf_joint_element != nullptr) {
      ParseJoint(sdf_joint_element, instance, tree, &frame_cache);
      sdf_joint_element = sdf_joint_element->GetNextElement("joint");
    }
  }

  FixDanglingLinks(instance, tree);
  return model_id;
#endif
}

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
