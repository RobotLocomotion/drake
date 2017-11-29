#include "drake/examples/double_pendulum/sdf_helpers.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "sdf/sdf.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsing/frame_cache.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace double_pendulum {

using multibody::parsing::FrameCache;

// RigidBodyTree model instance descriptor. Helpful
// to carry model name and instance id around.
struct ModelInstance {
  std::string name;
  int id;
};

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

// Parses a geometry from the given SDF element and adds a
// DrakeShapes::Geometry instance to the given element.
void ParseGeometry(sdf::ElementPtr sdf_geometry_element,
                   DrakeShapes::Element* element) {
  DRAKE_DEMAND(sdf_geometry_element != nullptr);
  DRAKE_DEMAND(sdf_geometry_element->GetName() == "geometry");
  DRAKE_DEMAND(element != nullptr);

  sdf::ElementPtr sdf_shape_element;
  if (sdf_geometry_element->HasElement("cylinder")) {
    sdf_shape_element = sdf_geometry_element->GetElement("cylinder");
    sdf::ElementPtr sdf_radius_element =
        sdf_shape_element->GetElement("radius");
    sdf::ElementPtr sdf_length_element =
        sdf_shape_element->GetElement("length");
    element->setGeometry(DrakeShapes::Cylinder(
        sdf_radius_element->Get<double>(), sdf_length_element->Get<double>()));
    return;
  }
  if (sdf_geometry_element->HasElement("box")) {
    sdf_shape_element = sdf_geometry_element->GetElement("box");
    sdf::ElementPtr sdf_size_element = sdf_shape_element->GetElement("size");
    const Vector3<double> xyz = ToVector(
        sdf_size_element->Get<ignition::math::Vector3d>());
    element->setGeometry(DrakeShapes::Box(xyz));
    return;
  }
  // TODO(hidmic): Support mesh and sphere geometries.
  DRAKE_ABORT_MSG("Unsupported geometry!");
}

// Parses a visual geometry from the given SDF element and adds a
// DrakeShapes::VisualElement instance to the given body.
void ParseVisual(sdf::ElementPtr sdf_visual_element, RigidBody<double>* body) {
  DRAKE_DEMAND(sdf_visual_element != nullptr);
  DRAKE_DEMAND(sdf_visual_element->GetName() == "visual");
  DRAKE_DEMAND(body != nullptr);

  const auto visual_name = sdf_visual_element->Get<std::string>("name");
  // Visual element frame's (E) pose in body frame (B).
  auto X_BE = Isometry3<double>::Identity();
  if (sdf_visual_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_visual_element->GetElement("pose");
    X_BE = ParsePose(sdf_pose_element);
  }

  DrakeShapes::VisualElement element(X_BE);
  sdf::ElementPtr sdf_geometry_element =
      sdf_visual_element->GetElement("geometry");
  ParseGeometry(sdf_geometry_element, &element);
  body->AddVisualElement(element);
}

// Parses a collision geometry from the given SDF element and adds a
// drake::multibody::collision::Element instance to the given body through the
// given tree.
void ParseCollision(sdf::ElementPtr sdf_collision_element,
                    RigidBody<double>* body,
                    RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(sdf_collision_element != nullptr);
  DRAKE_DEMAND(sdf_collision_element->GetName() == "collision");
  DRAKE_DEMAND(body != nullptr);
  DRAKE_DEMAND(tree != nullptr);

  const auto collision_name = sdf_collision_element->Get<std::string>("name");
  // Collision element's frame (E) pose in body frame (B).
  auto X_BE = Isometry3<double>::Identity();
  if (sdf_collision_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element =
        sdf_collision_element->GetElement("pose");
    X_BE = ParsePose(sdf_pose_element);
  }

  drake::multibody::collision::Element element(X_BE, body);
  sdf::ElementPtr sdf_geometry_element =
      sdf_collision_element->GetElement("geometry");
  ParseGeometry(sdf_geometry_element, &element);
  // Collision elements are added to the rigid body
  // tree, which then updates the corresponding rigid body.
  tree->addCollisionElement(element, *body, "default");
}

// Parses inertial information (mass, COM) from the given SDF
// element and updates the given body accordingly.
// TODO(hidmic): Parse inertial tensor as well.
void ParseInertial(sdf::ElementPtr sdf_inertial_element,
                   RigidBody<double>* body) {
  DRAKE_DEMAND(sdf_inertial_element != nullptr);
  DRAKE_DEMAND(sdf_inertial_element->GetName() == "inertial");
  DRAKE_DEMAND(body != nullptr);
  // Inertial frame's (I) pose in body frame (B).
  auto X_BI = Isometry3<double>::Identity();
  if (sdf_inertial_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_inertial_element->GetElement("pose");
    X_BI = ParsePose(sdf_pose_element);
  }
  // Center of mass p_BBcm measured and expressed in the body frame B,
  // as it is at the origin of the inertial frame (I).
  body->set_center_of_mass(X_BI.translation());

  if (sdf_inertial_element->HasElement("mass")) {
    sdf::ElementPtr sdf_mass_element = sdf_inertial_element->GetElement("mass");
    body->set_mass(sdf_mass_element->Get<double>());
  }

  // Define inertia tensor using the link mass on the link's frame,
  // and then transform it to the specified link's inertial frame.
  SquareTwistMatrix<double> itensor = SquareTwistMatrix<double>::Zero();
  itensor.block(3, 3, 3, 3) << body->get_mass() * Matrix3<double>::Identity();
  body->set_spatial_inertia(transformSpatialInertia(X_BI, itensor));
}

// Parses inertial, visual and collision details of a body, if any, from the
// given SDF element and adds a RigidBody instance to the given tree as part
// of the given model instance. The frame cache is updated with the body frame's
// pose (B) in its model frame (D).
void ParseLink(sdf::ElementPtr sdf_link_element,
               const ModelInstance& instance,
               RigidBodyTree<double>* tree,
               FrameCache<double>* frame_cache) {
  DRAKE_DEMAND(sdf_link_element != nullptr);
  DRAKE_DEMAND(sdf_link_element->GetName() == "link");
  DRAKE_DEMAND(tree != nullptr);
  DRAKE_DEMAND(frame_cache != nullptr);

  auto body = std::make_unique<RigidBody<double>>();
  const auto link_name = sdf_link_element->Get<std::string>("name");
  body->set_name(link_name);
  body->set_model_name(instance.name);
  body->set_model_instance_id(instance.id);

  // Body frame's (B) pose in model frame (D).
  auto X_DB = Isometry3<double>::Identity();
  if (sdf_link_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_link_element->GetElement("pose");
    X_DB = ParsePose(sdf_pose_element);
  }
  frame_cache->Update(instance.name, link_name, X_DB);

  if (sdf_link_element->HasElement("inertial")) {
    sdf::ElementPtr sdf_inertial_element =
        sdf_link_element->GetElement("inertial");
    ParseInertial(sdf_inertial_element, body.get());
  }

  if (sdf_link_element->HasElement("visual")) {
    sdf::ElementPtr sdf_visual_element = sdf_link_element->GetElement("visual");
    while (sdf_visual_element != nullptr) {
      ParseVisual(sdf_visual_element, body.get());
      sdf_visual_element = sdf_visual_element->GetNextElement("visual");
    }
  }

  if (sdf_link_element->HasElement("collision")) {
    sdf::ElementPtr sdf_collision_element =
        sdf_link_element->GetElement("collision");
    while (sdf_collision_element != nullptr) {
      ParseCollision(sdf_collision_element, body.get(), tree);
      sdf_collision_element =
          sdf_collision_element->GetNextElement("collision");
    }
  }
  tree->add_rigid_body(std::move(body));
}

// Parses a joint type from the given SDF element and instantiates the
// corresponding DrakeJoint instance. It is assumed that the parent body
// frame is already present in the frame cache.
std::unique_ptr<DrakeJoint>
ParseJointType(sdf::ElementPtr sdf_joint_element,
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

// Parses all bodies and joints of a model from the given SDF element
// and adds RigidBody and DrakeJoint instances to describe such model
// within the given tree.
//
// Returns the model instance id.
int ParseModel(sdf::ElementPtr sdf_model_element, RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(sdf_model_element != nullptr);
  DRAKE_DEMAND(sdf_model_element->GetName() == "model");
  DRAKE_DEMAND(tree != nullptr);

  // Define a model instance and a pose tree rooted at the model's frame (D).
  const auto model_name = sdf_model_element->Get<std::string>("name");
  const int model_id = tree->add_model_instance();
  ModelInstance instance{model_name, model_id};
  FrameCache<double> frame_cache(model_name);

  if (sdf_model_element->HasElement("link")) {
    sdf::ElementPtr sdf_link_element = sdf_model_element->GetElement("link");
    while (sdf_link_element != nullptr) {
      ParseLink(sdf_link_element, instance, tree, &frame_cache);
      sdf_link_element = sdf_link_element->GetNextElement("link");
    }
  }

  if (sdf_model_element->HasElement("joint")) {
    sdf::ElementPtr sdf_joint_element = sdf_model_element->GetElement("joint");
    while (sdf_joint_element != nullptr) {
      ParseJoint(sdf_joint_element, instance, tree, &frame_cache);
      sdf_joint_element = sdf_joint_element->GetNextElement("joint");
    }
  }

  FixDanglingLinks(instance, tree);
  return model_id;
}

int ParseModelFromFile(const std::string& sdf_path,
                       RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree != nullptr);
  sdf::SDFPtr parsed_sdf(new sdf::SDF());
  sdf::init(parsed_sdf);
  sdf::readFile(sdf_path, parsed_sdf);
  sdf::ElementPtr sdf_element = parsed_sdf->Root();
  DRAKE_DEMAND(sdf_element != nullptr);
  while (!sdf_element->HasElement("model")) {
    // To deal with both world and model SDFs, get next first
    // child element and check for model element existence.
    sdf_element = sdf_element->GetFirstElement();
    DRAKE_DEMAND(sdf_element != nullptr);
  }
  sdf::ElementPtr sdf_model_element = sdf_element->GetElement("model");
  return ParseModel(sdf_model_element, tree);
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
