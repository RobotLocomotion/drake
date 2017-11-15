#include "drake/multibody/parsing/sdf_parser.h"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include <sdf/sdf.hh>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/frame_cache.h"
#include "drake/multibody/parsing/sdf_link.h"
#include "drake/multibody/parsing/sdf_model.h"
#include "drake/multibody/parsing/sdf_spec.h"

namespace drake {
namespace multibody {
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
// element and updates the given link accordingly.
void ParseInertial(sdf::ElementPtr sdf_inertial_element, SdfLink* link) {
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
  // TODO(amcastro-tri): register inertial frame and set its pose in an
  // arbitrary measured-in frame.
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
void ParseLink(const sdf::ElementPtr sdf_link_element, SdfModel* sdf_model) {
  DRAKE_DEMAND(sdf_link_element != nullptr);
  DRAKE_DEMAND(sdf_link_element->GetName() == "link");
  DRAKE_DEMAND(sdf_model != nullptr);

  const auto link_name = sdf_link_element->Get<std::string>("name");
  SdfLink& sdf_link = sdf_model->AddLink(link_name);

  // Parse the pose of the link's frame (L) pose in the model frame (D).
  auto X_DL = Isometry3<double>::Identity();
  if (sdf_link_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_link_element->GetElement("pose");
    X_DL = ParsePose(sdf_pose_element);
  }
  // "Remember" the pose of this new link in the model frame D.
  sdf_model->CachePose(sdf_model->name(), link_name, X_DL);

  if (sdf_link_element->HasElement("inertial")) {
    sdf::ElementPtr sdf_inertial_element =
        sdf_link_element->GetElement("inertial");
    ParseInertial(sdf_inertial_element, &sdf_link);
  }

  // TODO(SeanCurtis-TRI): parse visual and collision elements.
}

// Parses a joint type from the given SDF element and reads its parameters
// accordingly into sdf_joint. It is assumed that the parent body frame is
// already present in the frame cache.
void ParseJointType(
    const sdf::ElementPtr sdf_joint_element,
    const SdfModel& sdf_model,
    SdfJoint* sdf_joint) {
  DRAKE_DEMAND(sdf_joint_element != nullptr);
  DRAKE_DEMAND(sdf_joint_element->GetName() == "joint");
  DRAKE_DEMAND(sdf_joint != nullptr);

  const auto joint_name = sdf_joint_element->Get<std::string>("name");
  const auto joint_type = sdf_joint_element->Get<std::string>("type");
  if (joint_type == "revolute") {
    sdf::ElementPtr sdf_axis_element = sdf_joint_element->GetElement("axis");
    sdf::ElementPtr sdf_xyz_element = sdf_axis_element->GetElement("xyz");
    // axis can be in either the "joint" frame M or in the model
    // frame D, depending on the boolean value for <use_parent_model_frame>.
    // use_parent_model_frame = false --> axis is in the joint frame M.
    // use_parent_model_frame = true --> axis is in the model frame D.
    Vector3<double> axis = ToVector(
        sdf_xyz_element->Get<ignition::math::Vector3d>());
    if (sdf_joint_element->HasElement("use_parent_model_frame")) {
      sdf::ElementPtr sdf_use_parent_frame_element =
          sdf_joint_element->GetElement("use_parent_model_frame");
      if (sdf_use_parent_frame_element->Get<bool>()) {
        // Axis of rotation is defined in the model frame (D).

        // Joint frame's (M) pose in model frame (D).
        const Isometry3<double> X_DM =
            sdf_model.GetPose(sdf_model.name(), joint_name);

        // Axis of rotation must be rotated back to the joint
        // frame (M), so the inverse of the rotational part of the
        // pose of the joint frame (M) in the model frame (D)
        // is applied.
        // Here we are doing: axis_M = R_MD * axis_D:
        axis = X_DM.inverse().linear() * axis;
      }
    }  // At this point, axis is expressed in the joint frame M.
    sdf_joint->set_axis(axis);
  } else {
    // TODO(amcastro-tri): Support prismatic and fixed joints.
    DRAKE_ABORT_MSG("Unsupported joint type!");
  }
}

void ParseJoint(sdf::ElementPtr sdf_joint_element, SdfModel* sdf_model) {
  DRAKE_DEMAND(sdf_joint_element != nullptr);
  DRAKE_DEMAND(sdf_joint_element->GetName() == "joint");
  DRAKE_DEMAND(sdf_model != nullptr);

  const auto joint_name = sdf_joint_element->Get<std::string>("name");
  sdf::ElementPtr sdf_parent_link_element =
      sdf_joint_element->GetElement("parent");
  const auto parent_link_name = sdf_parent_link_element->Get<std::string>();

  sdf::ElementPtr sdf_child_link_element =
      sdf_joint_element->GetElement("child");
  const auto child_link_name = sdf_child_link_element->Get<std::string>();

  // Joint frame's (M) pose in child link frame (L).
  auto X_LM = Isometry3<double>::Identity();
  if (sdf_joint_element->HasElement("pose")) {
    sdf::ElementPtr sdf_pose_element = sdf_joint_element->GetElement("pose");
    // Joint poses specified in SDF files are, by default, in the child link
    // frame (L), per SDF specification version 1.4.
    // See http://sdformat.org/spec?ver=1.4&elem=joint for details.
    X_LM = ParsePose(sdf_pose_element);
  }
  // "Remember" the pose of M in L.
  sdf_model->CachePose(child_link_name, joint_name, X_LM);

  const auto joint_type = sdf_joint_element->Get<std::string>("type");

  SdfJoint& sdf_joint = sdf_model->AddJoint(
      joint_name, parent_link_name, child_link_name, joint_type);

  ParseJointType(sdf_joint_element, *sdf_model, &sdf_joint);
}

// Parses all bodies and joints of a model from the given SDF element
// and adds RigidBody and DrakeJoint instances to describe such model
// within the given tree.
//
// Returns the model instance id.
void ParseModel(sdf::ElementPtr sdf_model_element, SdfSpec* spec) {
  DRAKE_DEMAND(sdf_model_element != nullptr);
  DRAKE_DEMAND(sdf_model_element->GetName() == "model");
  DRAKE_DEMAND(spec != nullptr);

  // Create a new SDF model.
  const auto model_name = sdf_model_element->Get<std::string>("name");
  SdfModel& sdf_model = spec->AddModel(model_name);

  if (sdf_model_element->HasElement("link")) {
    sdf::ElementPtr sdf_link_element = sdf_model_element->GetElement("link");
    while (sdf_link_element != nullptr) {
      ParseLink(sdf_link_element, &sdf_model);
      sdf_link_element = sdf_link_element->GetNextElement("link");
    }
  }

  if (sdf_model_element->HasElement("joint")) {
    sdf::ElementPtr sdf_joint_element = sdf_model_element->GetElement("joint");
    while (sdf_joint_element != nullptr) {
      ParseJoint(sdf_joint_element, &sdf_model);
      sdf_joint_element = sdf_joint_element->GetNextElement("joint");
    }
  }
}

std::unique_ptr<SdfSpec> ParseSdfModelFromFile(const std::string& sdf_path) {
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

  auto spec = std::make_unique<SdfSpec>(version);
  ParseModel(sdf_model_element, spec.get());
  return spec;
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
