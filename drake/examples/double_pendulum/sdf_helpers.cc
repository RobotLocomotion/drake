#include "drake/examples/double_pendulum/sdf_helpers.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "sdf/sdf.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/double_pendulum/pose_tree.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace examples {
namespace double_pendulum {

// RigidBodyTree model instance descriptor. Helpful
// to carry model name and instance id around.
struct ModelInstance {
  ModelInstance(const std::string& name, const int& id)
      : name(name), id(id) {}
  std::string name;
  int id;
};

Isometry3<double> ParsePose(const sdf::ElementPtr& pose) {
  auto native_pose = pose->Get<ignition::math::Pose3d>();
  Isometry3<double>::TranslationType translation(
      native_pose.Pos().X(),
      native_pose.Pos().Y(),
      native_pose.Pos().Z());
  Quaternion<double> rotation(
      native_pose.Rot().W(),
      native_pose.Rot().X(),
      native_pose.Rot().Y(),
      native_pose.Rot().Z());
  return translation * rotation;
}

void ParseGeometry(const sdf::ElementPtr& geometry,
                   DrakeShapes::Element* element) {
  sdf::ElementPtr shape;
  if (geometry->HasElement("cylinder")) {
    shape = geometry->GetElement("cylinder");
    sdf::ElementPtr radius = shape->GetElement("radius");
    sdf::ElementPtr length = shape->GetElement("length");
    element->setGeometry(DrakeShapes::Cylinder(
        radius->Get<double>(), length->Get<double>()));
    return;
  }
  if (geometry->HasElement("box")) {
    shape = geometry->GetElement("box");
    sdf::ElementPtr size = shape->GetElement("size");
    auto xyz = size->Get<ignition::math::Vector3d>();
    element->setGeometry(DrakeShapes::Box(
        Vector3<double>(xyz.X(), xyz.Y(), xyz.Z())));
    return;
  }
  DRAKE_ABORT_MSG("Unknown geometry!");
}

void ParseVisual(const sdf::ElementPtr& visual,
                 const ModelInstance& instance,
                 RigidBody<double>* body,
                 PoseTree<double>* pose_tree) {
  auto visual_name =
      visual->Get<std::string>("name");
  auto body_to_element_transform =
      Isometry3<double>::Identity();
  if (visual->HasElement("pose")) {
    sdf::ElementPtr pose =
      visual->GetElement("pose");
    body_to_element_transform = ParsePose(pose);
  }
  pose_tree->Update(body->get_name(), visual_name,
                    body_to_element_transform);

  DrakeShapes::VisualElement element(
      body_to_element_transform);
  sdf::ElementPtr geometry = visual->GetElement("geometry");
  ParseGeometry(geometry, &element);
  body->AddVisualElement(element);
}

void ParseCollision(const sdf::ElementPtr& collision,
                    const ModelInstance& instance,
                    RigidBody<double>* body,
                    RigidBodyTree<double>* tree,
                    PoseTree<double>* pose_tree) {
  auto body_to_element_transform =
      Isometry3<double>::Identity();
  if (collision->HasElement("pose")) {
    sdf::ElementPtr pose =
      collision->GetElement("pose");
    body_to_element_transform = ParsePose(pose);
  }
  auto collision_name =
      collision->Get<std::string>("name");
  pose_tree->Update(body->get_name(), collision_name,
                    body_to_element_transform);
  DrakeCollision::Element element(
      body_to_element_transform, body);
  sdf::ElementPtr geometry =
      collision->GetElement("geometry");
  ParseGeometry(geometry, &element);
  // Collision elements are added to the rigid body
  // tree, which then updates the corresponding rigid body.
  tree->addCollisionElement(element, *body, "default");
}

void ParseInertial(const sdf::ElementPtr& inertia,
                   RigidBody<double>* body) {
  auto body_to_inertia_transform =
      Isometry3<double>::Identity();
  if (inertia->HasElement("pose")) {
    sdf::ElementPtr pose = inertia->GetElement("pose");
    body_to_inertia_transform = ParsePose(pose);
  }
  // The center of mass is a position, thus we only
  // care about the translational part of the inertial
  // frame pose relative.
  body->set_center_of_mass(
      body_to_inertia_transform.translation());

  if (inertia->HasElement("mass")) {
    sdf::ElementPtr mass = inertia->GetElement("mass");
    body->set_mass(mass->Get<double>());
  }

  // Define inertia tensor using the link mass on the link's frame,
  // and then transform it to the specified link's inertial frame.
  SquareTwistMatrix<double> itensor = SquareTwistMatrix<double>::Zero();
  itensor.block(3, 3, 3, 3) << body->get_mass() * Matrix3<double>::Identity();
  body->set_spatial_inertia(
      transformSpatialInertia(body_to_inertia_transform, itensor));
}

void ParseLink(const sdf::ElementPtr& link,
               const ModelInstance& instance,
               RigidBodyTree<double>* tree,
               PoseTree<double>* pose_tree) {
  auto body = std::make_unique<RigidBody<double>>();
  auto link_name = link->Get<std::string>("name");
  body->set_name(link_name);
  body->set_model_name(instance.name);
  body->set_model_instance_id(instance.id);

  auto model_to_body_transform =
      Isometry3<double>::Identity();
  if (link->HasElement("pose")) {
    sdf::ElementPtr pose =
        link->GetElement("pose");
    model_to_body_transform = ParsePose(pose);
  }
  pose_tree->Update(instance.name, link_name,
                    model_to_body_transform);

  if (link->HasElement("inertial")) {
    sdf::ElementPtr inertia =
        link->GetElement("inertial");
    ParseInertial(inertia, body.get());
  }

  if (link->HasElement("visual")) {
    sdf::ElementPtr visual =
        link->GetElement("visual");
    while (visual != nullptr) {
      ParseVisual(visual, instance, body.get(), pose_tree);
      visual = visual->GetNextElement("visual");
    }
  }

  if (link->HasElement("collision")) {
    sdf::ElementPtr collision = link->GetElement("collision");
    while (collision != nullptr) {
      ParseCollision(collision, instance, body.get(), tree, pose_tree);
      collision = collision->GetNextElement("collision");
    }
  }
  tree->add_rigid_body(std::move(body));
}

std::unique_ptr<DrakeJoint>
ParseJointType(const sdf::ElementPtr& joint,
               const ModelInstance& instance,
               const RigidBody<double>* parent_body,
               const RigidBody<double>* child_body,
               const PoseTree<double>* pose_tree) {
  auto joint_name = joint->Get<std::string>("name");
  auto joint_type = joint->Get<std::string>("type");
  if (joint_type == "revolute") {
    sdf::ElementPtr axis = joint->GetElement("axis");
    sdf::ElementPtr xyz = axis->GetElement("xyz");
    auto native_axis_vector =
        xyz->Get<ignition::math::Vector3d>();
    Vector3<double> axis_vector(
        native_axis_vector.X(),
        native_axis_vector.Y(),
        native_axis_vector.Z());
    if (joint->HasElement("use_parent_model_frame")) {
      sdf::ElementPtr use_parent_frame =
          joint->GetElement("use_parent_model_frame");
      if (use_parent_frame->Get<bool>()) {
        // Axis of rotation is defined in the model frame.
        Isometry3<double> model_to_joint_transform =
            pose_tree->Transform(instance.name, joint_name);
        // Axis of rotation must be rotated back to the joint
        // frame, so the inverse of the rotational part of the
        // pose of the joint frame relative to the model frame
        // is applied.
        axis_vector =
            model_to_joint_transform.linear().inverse() * axis_vector;
      }
    }
    // Joints must be defined on the parent link's frame, thus
    // we provide the joint frame relative to the former.
    Isometry3<double> parent_body_to_joint_transform =
        pose_tree->Transform(parent_body->get_name(), joint_name);
    auto joint = std::make_unique<RevoluteJoint>(
        joint_name, parent_body_to_joint_transform, axis_vector);
    return std::move(joint);
  }
  DRAKE_ABORT_MSG("Unknown joint type!");
}

void ParseJoint(const sdf::ElementPtr& joint,
                const ModelInstance& instance,
                RigidBodyTree<double>* tree,
                PoseTree<double>* pose_tree) {
  auto joint_name = joint->Get<std::string>("name");

  sdf::ElementPtr parent_link = joint->GetElement("parent");
  auto parent_link_name = parent_link->Get<std::string>();
  RigidBody<double>* parent_body = tree->FindBody(
      parent_link_name, instance.name, instance.id);

  sdf::ElementPtr child_link = joint->GetElement("child");
  auto child_link_name = child_link->Get<std::string>();
  RigidBody<double>* child_body = tree->FindBody(
      child_link_name, instance.name, instance.id);

  // Joint frame transforms specified in SDF are,
  // by default, relative to the child link's frame.
  auto child_body_to_joint_transform =
      Isometry3<double>::Identity();
  if (joint->HasElement("pose")) {
    sdf::ElementPtr pose = joint->GetElement("pose");
    child_body_to_joint_transform = ParsePose(pose);
  }
  pose_tree->Update(child_link_name, joint_name,
                    child_body_to_joint_transform);

  // All child link's inertia, visual, and collision
  // elements reference frames must be this joint's frame.
  child_body->ApplyTransformToJointFrame(
      child_body_to_joint_transform.inverse());

  tree->transformCollisionFrame(
      child_body, child_body_to_joint_transform.inverse());

  // Update child link's parent and joint.
  child_body->setJoint(ParseJointType(
      joint, instance, parent_body, child_body, pose_tree));
  child_body->set_parent(parent_body);
}

void FixDanglingLinks(const ModelInstance& instance,
                      RigidBodyTree<double>* tree) {
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

void ParseModel(const sdf::ElementPtr& model,
                RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree != nullptr);
  DRAKE_DEMAND(model != nullptr);

  // Define a model instance and a pose tree rooted
  // at the model's frame.
  auto model_name = model->Get<std::string>("name");
  int model_id = tree->add_model_instance();
  ModelInstance instance(model_name, model_id);
  PoseTree<double> pose_tree(model_name);

  if (model->HasElement("link")) {
    sdf::ElementPtr link = model->GetElement("link");
    while (link != nullptr) {
      ParseLink(link, instance, tree, &pose_tree);
      link = link->GetNextElement("link");
    }
  }

  if (model->HasElement("joint")) {
    sdf::ElementPtr joint = model->GetElement("joint");
    while (joint != nullptr) {
      ParseJoint(joint, instance, tree, &pose_tree);
      joint = joint->GetNextElement("joint");
    }
  }

  FixDanglingLinks(instance, tree);
}

void ParseModelFromFile(const std::string& sdf_path,
                        RigidBodyTree<double>* tree) {
  sdf::SDFPtr parsed_sdf(new sdf::SDF());
  sdf::init(parsed_sdf);
  sdf::readFile(sdf_path, parsed_sdf);
  sdf::ElementPtr sdf_element = parsed_sdf->Root();
  sdf::ElementPtr model_element =
      sdf_element->GetElement("model");
  ParseModel(model_element, tree);
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
