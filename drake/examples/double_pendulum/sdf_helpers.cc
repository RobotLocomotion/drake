#include "drake/examples/double_pendulum/sdf_helpers.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "sdf/sdf.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/double_pendulum/frame_cache.h"
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
  std::string name{};
  int id{};
};


Vector3<double> ToVector(const ignition::math::Vector3d& vector) {
  return Vector3<double>(vector.X(), vector.Y(), vector.Z());
}

Isometry3<double> ToIsometry(const ignition::math::Pose3d& pose) {
  const Isometry3<double>::TranslationType translation(ToVector(pose.Pos()));
  const Quaternion<double> rotation(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());
  return translation * rotation;
}

Isometry3<double> ParsePose(sdf::ElementPtr pose) {
  return ToIsometry(pose->Get<ignition::math::Pose3d>());
}

void ParseGeometry(sdf::ElementPtr geometry,
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
    const auto xyz = size->Get<ignition::math::Vector3d>();
    element->setGeometry(DrakeShapes::Box(ToVector(xyz)));
    return;
  }
  DRAKE_ABORT_MSG("Unknown geometry!");
}

void ParseVisual(sdf::ElementPtr visual,
                 RigidBody<double>* body,
                 FrameCache<double>* frame_cache) {
  const auto visual_name = visual->Get<std::string>("name");
  // Visual element frame's (E) pose in body frame (B).
  auto X_BE = Isometry3<double>::Identity();
  if (visual->HasElement("pose")) {
    sdf::ElementPtr pose = visual->GetElement("pose");
    X_BE = ParsePose(pose);
  }
  frame_cache->Update(body->get_name(), visual_name, X_BE);

  DrakeShapes::VisualElement element(X_BE);
  sdf::ElementPtr geometry = visual->GetElement("geometry");
  ParseGeometry(geometry, &element);
  body->AddVisualElement(element);
}

void ParseCollision(sdf::ElementPtr collision,
                    RigidBody<double>* body,
                    RigidBodyTree<double>* tree,
                    FrameCache<double>* frame_cache) {
  const auto collision_name = collision->Get<std::string>("name");
  // Collision element's frame (E) pose in body frame (B).
  auto X_BE = Isometry3<double>::Identity();
  if (collision->HasElement("pose")) {
    sdf::ElementPtr pose = collision->GetElement("pose");
    X_BE = ParsePose(pose);
  }
  frame_cache->Update(body->get_name(), collision_name, X_BE);
  DrakeCollision::Element element(X_BE, body);
  sdf::ElementPtr geometry = collision->GetElement("geometry");
  ParseGeometry(geometry, &element);
  // Collision elements are added to the rigid body
  // tree, which then updates the corresponding rigid body.
  tree->addCollisionElement(element, *body, "default");
}

void ParseInertial(sdf::ElementPtr inertia,
                   RigidBody<double>* body) {
  // Inertial frame's (I) pose in body frame (B).
  auto X_BI = Isometry3<double>::Identity();
  if (inertia->HasElement("pose")) {
    const sdf::ElementPtr pose = inertia->GetElement("pose");
    X_BI = ParsePose(pose);
  }
  // The center of mass is a position, thus we only
  // care about the translational part.
  body->set_center_of_mass(X_BI.translation());

  if (inertia->HasElement("mass")) {
    sdf::ElementPtr mass = inertia->GetElement("mass");
    body->set_mass(mass->Get<double>());
  }

  // Define inertia tensor using the link mass on the link's frame,
  // and then transform it to the specified link's inertial frame.
  SquareTwistMatrix<double> itensor = SquareTwistMatrix<double>::Zero();
  itensor.block(3, 3, 3, 3) << body->get_mass() * Matrix3<double>::Identity();
  body->set_spatial_inertia(transformSpatialInertia(X_BI, itensor));
}

void ParseLink(sdf::ElementPtr link,
               const ModelInstance& instance,
               RigidBodyTree<double>* tree,
               FrameCache<double>* frame_cache) {
  auto body = std::make_unique<RigidBody<double>>();
  const auto link_name = link->Get<std::string>("name");
  body->set_name(link_name);
  body->set_model_name(instance.name);
  body->set_model_instance_id(instance.id);

  // Body frame's (B) pose in model frame (M).
  auto X_MB = Isometry3<double>::Identity();
  if (link->HasElement("pose")) {
    sdf::ElementPtr pose = link->GetElement("pose");
    X_MB = ParsePose(pose);
  }
  frame_cache->Update(instance.name, link_name, X_MB);

  if (link->HasElement("inertial")) {
    sdf::ElementPtr inertia = link->GetElement("inertial");
    ParseInertial(inertia, body.get());
  }

  if (link->HasElement("visual")) {
    sdf::ElementPtr visual = link->GetElement("visual");
    while (visual != nullptr) {
      ParseVisual(visual, body.get(), frame_cache);
      visual = visual->GetNextElement("visual");
    }
  }

  if (link->HasElement("collision")) {
    sdf::ElementPtr collision = link->GetElement("collision");
    while (collision != nullptr) {
      ParseCollision(collision, body.get(), tree, frame_cache);
      collision = collision->GetNextElement("collision");
    }
  }
  tree->add_rigid_body(std::move(body));
}

std::unique_ptr<DrakeJoint>
ParseJointType(sdf::ElementPtr joint,
               const ModelInstance& instance,
               const RigidBody<double>* parent_body,
               const FrameCache<double>* frame_cache) {
  const auto joint_name = joint->Get<std::string>("name");
  const auto joint_type = joint->Get<std::string>("type");
  if (joint_type == "revolute") {
    sdf::ElementPtr axis = joint->GetElement("axis");
    sdf::ElementPtr xyz = axis->GetElement("xyz");
    Vector3<double> axis_vector = ToVector(
        xyz->Get<ignition::math::Vector3d>());
    if (joint->HasElement("use_parent_model_frame")) {
      sdf::ElementPtr use_parent_frame =
          joint->GetElement("use_parent_model_frame");
      if (use_parent_frame->Get<bool>()) {
        // Axis of rotation is defined in the model frame.

        // Joint frame's (J) pose in model frame (M).
        const Isometry3<double> X_MJ =
            frame_cache->Transform(instance.name, joint_name);
        // Axis of rotation must be rotated back to the joint
        // frame, so the inverse of the rotational part of the
        // pose of the joint frame (J) in the model frame (M)
        // is applied.
        axis_vector = X_MJ.linear().inverse() * axis_vector;
      }
    }
    // Joints must be defined on the parent link's frame, thus we
    // provide the joint frame's (J) pose in the parent body frame (P).
    const Isometry3<double> X_PJ =
        frame_cache->Transform(parent_body->get_name(), joint_name);
    auto joint = std::make_unique<RevoluteJoint>(joint_name, X_PJ, axis_vector);
    return std::move(joint);
  }
  DRAKE_ABORT_MSG("Unknown joint type!");
}

void ParseJoint(sdf::ElementPtr joint,
                const ModelInstance& instance,
                RigidBodyTree<double>* tree,
                FrameCache<double>* frame_cache) {
  const auto joint_name = joint->Get<std::string>("name");

  sdf::ElementPtr parent_link = joint->GetElement("parent");
  const auto parent_link_name = parent_link->Get<std::string>();
  RigidBody<double>* parent_body = tree->FindBody(
      parent_link_name, instance.name, instance.id);

  sdf::ElementPtr child_link = joint->GetElement("child");
  const auto child_link_name = child_link->Get<std::string>();
  RigidBody<double>* child_body = tree->FindBody(
      child_link_name, instance.name, instance.id);

  // Joint frame's (J) pose in child body frame (C).
  auto X_CJ = Isometry3<double>::Identity();
  if (joint->HasElement("pose")) {
    sdf::ElementPtr pose = joint->GetElement("pose");
    // Joint poses specified in SDF files are, by default,
    // in the child link frame.
    X_CJ = ParsePose(pose);
  }
  frame_cache->Update(child_link_name, joint_name, X_CJ);

  // All child link's inertia, visual, and collision
  // elements reference frames must be this joint's frame.
  child_body->ApplyTransformToJointFrame(X_CJ.inverse());

  tree->transformCollisionFrame(child_body, X_CJ.inverse());

  // Update child link's parent and joint.
  child_body->setJoint(ParseJointType(
      joint, instance, parent_body, frame_cache));
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

int ParseModel(sdf::ElementPtr model, RigidBodyTree<double>* tree) {
  DRAKE_DEMAND(tree != nullptr);
  DRAKE_DEMAND(model != nullptr);

  // Define a model instance and a pose tree rooted at the model's frame (M).
  const auto model_name = model->Get<std::string>("name");
  const int model_id = tree->add_model_instance();
  ModelInstance instance(model_name, model_id);
  FrameCache<double> frame_cache(model_name);

  if (model->HasElement("link")) {
    sdf::ElementPtr link = model->GetElement("link");
    while (link != nullptr) {
      ParseLink(link, instance, tree, &frame_cache);
      link = link->GetNextElement("link");
    }
  }

  if (model->HasElement("joint")) {
    sdf::ElementPtr joint = model->GetElement("joint");
    while (joint != nullptr) {
      ParseJoint(joint, instance, tree, &frame_cache);
      joint = joint->GetNextElement("joint");
    }
  }

  FixDanglingLinks(instance, tree);
  return model_id;
}

int ParseModelFromFile(const std::string& sdf_path,
                       RigidBodyTree<double>* tree) {
  sdf::SDFPtr parsed_sdf(new sdf::SDF());
  sdf::init(parsed_sdf);
  sdf::readFile(sdf_path, parsed_sdf);
  sdf::ElementPtr sdf_element = parsed_sdf->Root();
  DRAKE_DEMAND(sdf_element != nullptr);
  while (!sdf_element->HasElement("model")) {
    sdf_element = sdf_element->GetFirstElement();
    DRAKE_DEMAND(sdf_element != nullptr);
  }
  sdf::ElementPtr model_element = sdf_element->GetElement("model");
  return ParseModel(model_element, tree);
}

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
