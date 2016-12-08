#include "drake/multibody/parser_common.h"

#include <cstdlib>
#include <string>

#include "spruce.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"
#include "drake/thirdParty/bsd/tinydir/tinydir.h"

using std::cerr;
using std::endl;
using std::getenv;
using std::istringstream;
using std::make_pair;
using std::map;
using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace drake {
namespace parsers {

using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kRollPitchYaw;
using drake::multibody::joints::kQuaternion;


namespace {
// Searches for key @package in @package_map. If the key exists, this saves the
// associated value in the string pointed to by @package_path and then returns
// true. It returns false otherwise.
bool GetPackagePath(const string& package,
    const map<string, string>& package_map, string* package_path) {
  auto iter = package_map.find(package);
  if (iter != package_map.end()) {
    *package_path = iter->second;
    return true;
  } else {
    drake::log()->warn("Warning: Couldn't find package '{}' in the supplied "
                       "package path.", package);
    return false;
  }
}
}  // anonymous namespace

string ResolveFilename(const string& filename,
                       const map<string, string>& package_map,
                       const string& root_dir) {
  spruce::path mesh_filename_s;
  spruce::path raw_filename_s(filename);

  auto split_filename = raw_filename_s.split();

  if (split_filename.front() == "package:") {
    string package_path_string;
    if (GetPackagePath(split_filename.at(2), package_map,
        &package_path_string)) {
      spruce::path package_path_s = spruce::path(package_path_string);
      mesh_filename_s = package_path_s;

      auto split_raw = raw_filename_s.split();
      for (int i = 1; i < static_cast<int>(split_raw.size()) - 2; ++i) {
        mesh_filename_s.append(split_raw.at(i + 2));
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
    bool dirIsRelative =
        !(normalized_root_dir.size() >= 1 && normalized_root_dir[0] == '/');
    if (dirIsRelative) {
      mesh_filename_s = spruce::path();
      mesh_filename_s.setAsCurrent();
      mesh_filename_s.append(normalized_root_dir);
    } else {
      mesh_filename_s = spruce::path(normalized_root_dir);
    }

    mesh_filename_s.append(filename);
  }
  if (!mesh_filename_s.exists()) {
    drake::log()->warn("File '{}' could not be found.",
                       mesh_filename_s.getStr());
    drake::log()->warn("Mesh '{}' could not be resolved and will be ignored by "
                       "Drake.", filename);
    return string();
  }
  return mesh_filename_s.getStr();
}

void AddPackage(const string& name, const string& path,
    PackageMap* package_map) {
  DRAKE_DEMAND(package_map != nullptr);
  DRAKE_DEMAND(package_map->find(name) == package_map->end());
  package_map->insert(make_pair(name, path));
}

namespace {

// Searches in directory @p path for files called "package.xml".
// Adds the package name specified in package.xml and the path to the
// package to @p package_map.
void CrawlForPackages(const string& path, PackageMap* package_map) {
  string token, t;
  std::string directory_path = path;

  // Removes trailing "/" if it exists.
  if (directory_path.length() > 0) {
    std::string::iterator it = directory_path.end() - 1;
    if (*it == '/')
      directory_path.erase(it);
  }

  istringstream iss(directory_path);
  const string target_filename("package.xml");
  const char pathsep = ':';

  while (getline(iss, token, pathsep)) {
    tinydir_dir dir;
    if (tinydir_open(&dir, token.c_str()) < 0) {
      cerr << "Unable to open directory: " << token << endl;
      continue;
    }

    while (dir.has_next) {
      tinydir_file file;
      tinydir_readfile(&dir, &file);

      // Skips hidden directories (including "." and "..").
      if (file.is_dir && (file.name[0] != '.')) {
        CrawlForPackages(file.path, package_map);
      } else if (file.name == target_filename) {
        // Parses the package.xml file to find the name of the package.
        string package_name;

        {
          string file_name = string(file.path);

          XMLDocument xml_doc;
          xml_doc.LoadFile(file_name.data());
          if (xml_doc.ErrorID()) {
            throw runtime_error("parser_common.cc: CrawlForPackages(): "
                "Failed to parse XML in file \"" + file_name + "\".\n" +
                xml_doc.ErrorName());
          }

          XMLElement* package_node = xml_doc.FirstChildElement("package");
          if (!package_node) {
            throw runtime_error("parser_common.cc: CrawlForPackages(): "
                "ERROR: XML file \"" + file_name + "\" does not contain "
                "element <package>.");
          }

          XMLElement* name_node = package_node->FirstChildElement("name");
          if (!name_node) {
            throw runtime_error("parser_common.cc: CrawlForPackages(): "
                "ERROR: <package> element does not contain element <name> "
                "(XML file \"" + file_name + "\").");
          }

          package_name = name_node->FirstChild()->Value();
        }

        spruce::path mypath_s(file.path);

        // Don't overwrite entries in the map.
        auto package_iter = package_map->find(package_name);
        if (package_iter == package_map->end()) {
          package_map->insert(
              make_pair(package_name, mypath_s.root().append("/")));
        } else {
          cerr << "parser_common.cc: CrawlForPackages: WARNING: Package \""
               << package_name
               << "\" was found more than once in the search "
               << "space." << endl;
        }
      }
      tinydir_next(&dir);
    }
    tinydir_close(&dir);
  }
}

}  // namespace

void PopulateMapFromFolder(const string& path, PackageMap* package_map) {
  CrawlForPackages(path, package_map);
}

void PopulateMapFromEnvironment(const string environment_variable,
    PackageMap* package_map) {
  const char* path_char = getenv(environment_variable.c_str());
  DRAKE_DEMAND(path_char);
  string path = string(path_char);
  CrawlForPackages(path, package_map);
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
          == string(RigidBodyTree<double>::kWorldName)) {
      if (!weld_to_frame->has_as_rigid_body(nullptr)) {
        throw runtime_error(
            "RigidBodyTree::AddFloatingJoint: "
            "Attempted to weld robot to the world while specifying a body "
            "link!");
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
          throw runtime_error("unknown floating base type");
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
