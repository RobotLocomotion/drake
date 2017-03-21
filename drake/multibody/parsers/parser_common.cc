#include "drake/multibody/parsers/parser_common.h"

#include <string>
#include <utility>

#include "spruce.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"

using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace parsers {

using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;

std::string GetFullPath(const std::string& file_name) {
  std::string result = file_name;
  if (result.empty()) {
    throw std::runtime_error("drake::parsers::GetFullPath: ERROR: file_name is "
                             "empty.");
  }

  const std::string prefix = "/";
  if (result.substr(0, prefix.size()) == prefix) {
    // The specified file is already an absolute path. The following code
    // verifies that the file exists.
    spruce::path path(file_name);
    if (!path.isFile()) {
      throw std::runtime_error("drake::parsers::GetFullPath: ERROR: "
          "file_name \"" + file_name + "\" is not a file.");
    }
  } else {
    // The specified file is a relative path. The following code obtains the
    // full path and verifies that the file exists.
    spruce::path path(".");
    path.setAsCurrent();
    path.append(file_name);
    if (path.isFile()) {
      result = path.getStr();
    } else {
      throw std::runtime_error("drake::parsers::GetFullPath: ERROR: "
          "file_name \"" + file_name + "\" is not a file or does not exist.");
    }
  }
  return result;
}

namespace {
// Searches for key p package in package_map. If the key exists, this saves the
// associated value in the string pointed to by package_path and then returns
// true. It returns false otherwise.
bool GetPackagePath(const string& package, const PackageMap& package_map,
                    string* package_path) {
  if (package_map.Contains(package)) {
    *package_path = package_map.GetPath(package);
    return true;
  } else {
    drake::log()->warn("Couldn't find package '{}' in the supplied package"
                       "path.", package);
    return false;
  }
}
}  // anonymous namespace

// The unit test that most directly covers this method is:
// drake/multibody/parsers/test/urdf_parser_test/urdf_parser_test.cc.
string ResolveFilename(const string& filename, const PackageMap& package_map,
                       const string& root_dir) {
  spruce::path mesh_filename_spruce;
  spruce::path raw_filename_spruce(filename);

  vector<std::string> split_filename = raw_filename_spruce.split();

  if (split_filename.front() == "package:") {
    // A correctly formatted filename is:
    //
    //   package://package_name/bar/baz/model.xyz
    //
    // Thus, index 0 contains "package", index 1 contains "", and index 2
    // contains the package name. Furthermore, since the model file must follow
    // the package name, there must be at least 4 tokens.
    const int kMinNumTokens = 4;
    const int kPackageNameIndex = 2;
    DRAKE_DEMAND(split_filename.size() >= kMinNumTokens);

    string package_path_string;
    if (GetPackagePath(split_filename.at(kPackageNameIndex), package_map,
        &package_path_string)) {
      mesh_filename_spruce = spruce::path(package_path_string);

      auto split_raw = raw_filename_spruce.split();
      // The following loop starts at index 3 to skip the "package", "", and
      // [package name] tokens as described above.
      for (int i = kPackageNameIndex + 1;
          i < static_cast<int>(split_raw.size()); ++i) {
        mesh_filename_spruce.append(split_raw.at(i));
      }
    } else {
      drake::log()->warn("Mesh '{}' could not be resolved and will be ignored "
                         "by Drake. If you don't want it to be ignored, please "
                         "include it in the package map.", filename);
      return string();
    }
  } else {
    string normalized_root_dir = spruce::path(root_dir).getStr();

    // If root_dir is a relative path, convert it to an absolute path.
    bool dir_is_relative =
        !(normalized_root_dir.size() >= 1 && normalized_root_dir[0] == '/');
    if (dir_is_relative) {
      mesh_filename_spruce = spruce::path();
      mesh_filename_spruce.setAsCurrent();
      mesh_filename_spruce.append(normalized_root_dir);
    } else {
      mesh_filename_spruce = spruce::path(normalized_root_dir);
    }

    mesh_filename_spruce.append(filename);
  }
  if (!mesh_filename_spruce.exists()) {
    drake::log()->warn("File '{}' could not be found.",
                       mesh_filename_spruce.getStr());
    drake::log()->warn("Mesh '{}' could not be resolved and will be ignored by "
                       "Drake.", filename);
    return string();
  }
  return mesh_filename_spruce.getStr();
}

int AddFloatingJoint(
    const FloatingBaseType floating_base_type,
    const vector<int>& body_indices,
    const std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    const PoseMap* pose_map,
    RigidBodyTree<double>* tree) {
  string floating_joint_name;
  RigidBody<double>* weld_to_body{nullptr};
  Eigen::Isometry3d transform_to_world;

  if (weld_to_frame == nullptr) {
    // If weld_to_frame is not specified, weld the newly added model(s) to the
    // world with zero offset.
    weld_to_body = tree->bodies[0].get();
    floating_joint_name = "base";
    transform_to_world = Eigen::Isometry3d::Identity();
  } else {
    // If weld_to_frame is specified and the model is being welded to the world,
    // ensure the "body" variable within weld_to_frame is nullptr. Then, only
    // use the transform_to_body variable within weld_to_frame to initialize
    // the robot at the desired location in the world.
    if (weld_to_frame->get_name()
          == string(RigidBodyTreeConstants::kWorldName)) {
      if (!weld_to_frame->has_as_rigid_body(nullptr)) {
        throw runtime_error(
            "AddFloatingJoint: Attempted to weld robot to the world while "
            "specifying a body link!");
      }
      weld_to_body = tree->bodies[0].get();  // the world's body
      floating_joint_name = "base";
    } else {
      weld_to_body = weld_to_frame->get_mutable_rigid_body();
      floating_joint_name = "weld";
    }
    transform_to_world = weld_to_frame->get_transform_to_body();
  }

  int num_floating_joints_added = 0;

  for (auto i : body_indices) {
    if (tree->bodies[i]->get_parent() == nullptr) {
      // The following code connects the parent-less link to the rigid body tree
      // using a floating joint.
      tree->bodies[i]->set_parent(weld_to_body);

      Eigen::Isometry3d transform_to_model = Eigen::Isometry3d::Identity();
      if (pose_map != nullptr &&
          pose_map->find(tree->bodies[i]->get_name()) != pose_map->end())
        transform_to_model = pose_map->at(tree->bodies[i]->get_name());

      switch (floating_base_type) {
        case kFixed: {
          unique_ptr<DrakeJoint> joint(new FixedJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case kRollPitchYaw: {
          unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case kQuaternion: {
          unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          tree->bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        default:
          throw runtime_error("Unknown floating base type.");
      }
    }
  }

  if (num_floating_joints_added == 0) {
    // TODO(liang.fok) Handle this by disconnecting one of the internal nodes,
    // replacing the disconnected joint with a loop joint, and then connecting
    // the newly freed body (i.e., the body without a joint) to the world.
    throw runtime_error(
        "No root bodies found. Every body referenced by the supplied list of "
        "body indices has a joint connecting it to some other body.  This will "
        "result in RigidBodyTree::compile() looping indefinitely. This "
        "scenario currently not supported in Drake.");
  }

  return num_floating_joints_added;
}

}  // namespace parsers
}  // namespace drake
