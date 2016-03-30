
#include <fstream>
#include <sstream>
#include <string>

#include "drake/Path.h"
#include "drake/thirdParty/tinydir/tinydir.h"
#include "drake/util/drakeGeometryUtil.h"
#include "xmlUtil.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;

// only writes values if they exist
bool parseVectorAttribute(tinyxml2::XMLElement* node,
                          const char* attribute_name, Eigen::Vector3d& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorAttribute(tinyxml2::XMLElement* node,
                          const char* attribute_name, Eigen::Vector4d& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

bool parseVectorValue(tinyxml2::XMLElement* node, const char* element_name,
                      Eigen::Vector3d& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    std::stringstream s(elnode->FirstChild()->Value());
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorValue(tinyxml2::XMLElement* node, const char* element_name,
                      Eigen::Vector4d& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    std::stringstream s(elnode->FirstChild()->Value());
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

bool parseStringValue(tinyxml2::XMLElement* node, const char* element_name,
                      std::string& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    val = elnode->FirstChild()->Value();
    return true;
  }
  return false;
}

void originAttributesToTransform(tinyxml2::XMLElement* node,
                                 Eigen::Isometry3d& T) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Zero();

  parseVectorAttribute(node, "xyz", xyz);
  parseVectorAttribute(node, "rpy", rpy);

  T.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

void poseValueToTransform(tinyxml2::XMLElement* node, const PoseMap& pose_map,
                          Eigen::Isometry3d& T,
                          const Eigen::Isometry3d& T_default_frame) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Zero();
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    std::stringstream s(strval);
    s >> xyz(0) >> xyz(1) >> xyz(2) >> rpy(0) >> rpy(1) >> rpy(2);
  }

  T.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;

  const char* attr = node->Attribute("frame");
  if (attr && strlen(attr) > 0) {
    std::string frame;
    std::stringstream s(attr);
    s >> frame;
    assert(0 && "this has not been tested yet");  // and could cause problems
                                                  // with the default pose
                                                  // assumptions in the sdf
                                                  // parser, which I simply had
                                                  // to guess
    Eigen::Isometry3d T_frame =
        pose_map.at(frame);  // will throw an exception if the frame is not
                             // found.  that is the desired behavior.
    T = T_frame * T;
  } else {
    T = T_default_frame * T;
  }
}

namespace {
void searchDirectory(map<string, string>& package_map, string path) {
#if defined(WIN32) || defined(WIN64)
  const char pathsep = ';';
#else
  const char pathsep = ':';
#endif

  string token, t;
  istringstream iss(path);
  const std::string target_filename("package.xml");

  while (getline(iss, token, pathsep)) {
    tinydir_dir dir;
    if (tinydir_open(&dir, token.c_str()) < 0) {
      std::cerr << "Unable to open directory: " << token << std::endl;
      continue;
    }

    while (dir.has_next) {
      tinydir_file file;
      tinydir_readfile(&dir, &file);

      // Skip hidden directories (including, importantly, "." and "..").
      if (file.is_dir && (file.name[0] != '.')) {
        searchDirectory(package_map, file.path);
      } else if (file.name == target_filename) {
        spruce::path mypath_s(file.path);
        auto path_split = mypath_s.split();
        if (path_split.size() > 2) {
          string package = path_split.at(path_split.size() - 2);
          auto package_iter = package_map.find(package);
          // Don't overwrite entries in the map
          if (package_iter == package_map.end()) {
            package_map.insert(make_pair(package, mypath_s.root().append("/")));
          }
        }
      }
      tinydir_next(&dir);
    }
    tinydir_close(&dir);
  }
}
}

void populatePackageMap(map<string, string>& package_map) {
  searchDirectory(package_map, Drake::getDrakePath());

  char* cstrpath = getenv("ROS_ROOT");
  if (cstrpath) searchDirectory(package_map, cstrpath);

  cstrpath = getenv("ROS_PACKAGE_PATH");
  if (cstrpath) searchDirectory(package_map, cstrpath);
}

namespace {
bool rospack(const string& package, const map<string, string>& package_map,
             string& package_path) {
  // my own quick and dirty implementation of the rospack algorithm (based on my
  // matlab version in rospack.m)
  auto iter = package_map.find(package);
  if (iter != package_map.end()) {
    package_path = iter->second;
    return true;
  } else {
    cerr << "Warning: Couldn't find package '" << package
         << "' in ROS_ROOT, ROS_PACKAGE_PATH, or user supplied package map"
         << endl;
    return false;
  }
}
}

string resolveFilename(const string& filename,
                       const map<string, string>& package_map,
                       const string& root_dir) {
  spruce::path mesh_filename_s;
  spruce::path raw_filename_s(filename);

  auto split_filename = raw_filename_s.split();

  if (split_filename.front() == "package:") {
    string package_path_string;
    if (rospack(split_filename.at(2), package_map, package_path_string)) {
      spruce::path package_path_s = spruce::path(package_path_string);
      mesh_filename_s = package_path_s;

      auto split_raw = raw_filename_s.split();
      for (int i = 1; i < split_raw.size() - 2; ++i) {
        mesh_filename_s.append(split_raw.at(i + 2));
      }
    } else {
      cerr << "Warning: Mesh '" << filename
           << "' could not be resolved and will be ignored by Drake." << endl;
      return string();
    }
  } else {
    mesh_filename_s = spruce::path(root_dir);
    mesh_filename_s.append(filename);
  }
  if (!mesh_filename_s.exists()) {
    cerr << "Warning: File '" << mesh_filename_s.getStr()
         << "' could not be found." << endl;
    cerr << "Warning: Mesh '" << filename
         << "' could not be resolved and will be ignored by Drake." << endl;
    return string();
  }
  return mesh_filename_s.getStr();
}
