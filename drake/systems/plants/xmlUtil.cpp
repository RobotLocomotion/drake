#include "drake/systems/plants/xmlUtil.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "spruce.hh"

#include "drake/common/drake_path.h"
#include "drake/common/drake_assert.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/thirdParty/bsd/tinydir/tinydir.h"

using std::cerr;
using std::endl;
using std::istringstream;
using std::map;
using std::string;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

void ParseThreeVectorValue(const char* strval, Eigen::Vector3d* val) {
  if (val == nullptr) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"val\" is null.");
  }

  // Handles the case where strval is a nullptr.
  if (!strval) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"strval\" is null.");
  }

  // Handles the case where strval is an empty strval.
  if (std::string(strval) == "") {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"strval\" is empty.");
  }

  std::stringstream ss(strval);
  std::string token;
  ss >> token;

  (*val)[0] = StringToDouble(token);

  // Handles the case where strval is a single scalar value.
  if (!ss.good()) {
    (*val)[1] = (*val)[0];
    (*val)[2] = (*val)[0];
    return;
  }

  ss >> token;
  (*val)[1] = StringToDouble(token);

  // Handles the case where strval is a 2 vector.
  if (!ss.good()) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: A 2 vector was supplied.");
  }

  ss >> token;
  (*val)[2] = StringToDouble(token);

  // Handles the case where strval is vector that's longer than 3.
  if (ss.good()) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: A vector with more than three "
        "elements was supplied.");
  }
}

double StringToDouble(const string& str) {
  std::size_t num_chars = 0;
  double result = std::stod(str, &num_chars);

  // Verifies that there are no additional characters after the double value.
  if (str.size() != num_chars) {
    throw std::invalid_argument(
        "ERROR: Double value contained additional characters after the "
        "number.");
  }

  return result;
}

void ParseThreeVectorValue(const tinyxml2::XMLElement* node,
                           Eigen::Vector3d* val) {
  if (node) {
    ParseThreeVectorValue(node->FirstChild()->Value(), val);
  } else {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"node\" is null.");
  }
}

void ParseThreeVectorValue(const tinyxml2::XMLElement* node,
                           const char* element_name, Eigen::Vector3d* val) {
  if (!node || !element_name) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"node\" and/or parameter "
        "\"element_name\" is null.");
  } else {
    const tinyxml2::XMLElement* child_node =
        node->FirstChildElement(element_name);
    if (child_node == nullptr) {
      throw std::invalid_argument(
          "ERROR: ParseThreeVectorValue: Element \"" +
          std::string(element_name) + "\" not found.");
    }
    ParseThreeVectorValue(child_node, val);
  }
}

void ParseThreeVectorAttribute(const tinyxml2::XMLElement* node,
                               const char* attribute_name,
                               Eigen::Vector3d* val) {
  if (!node || !attribute_name) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorAttribute: Parameter"
        "\"node\" and/or parameter \"attribute_name\" is null.");
  } else {
    const char* scale = node->Attribute(attribute_name);
    if (scale == nullptr) {
      throw std::invalid_argument(
          "ERROR: ParseThreeVectorAttribute: Attribute \"" +
          std::string(attribute_name) + "\" not found.");
    }
    ParseThreeVectorValue(scale, val);
  }
}

// only writes values if they exist
bool parseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::Vector3d& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::Vector4d& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

bool parseVectorValue(tinyxml2::XMLElement* node, const char* element_name,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      std::string& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    val = elnode->FirstChild()->Value();
    return true;
  }
  return false;
}

void originAttributesToTransform(
    tinyxml2::XMLElement* node,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Isometry3d& T) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Zero();

  parseVectorAttribute(node, "xyz", xyz);
  parseVectorAttribute(node, "rpy", rpy);

  T.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

void poseValueToTransform(tinyxml2::XMLElement* node, const PoseMap& pose_map,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::Isometry3d& T,
                          const Eigen::Isometry3d& T_default_frame) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero(), xyz = Eigen::Vector3d::Zero();
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    std::stringstream s(strval);
    s >> xyz(0) >> xyz(1) >> xyz(2) >> rpy(0) >> rpy(1) >> rpy(2);
  }

  T.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;

  const char* attr = node->Attribute("frame");
  if (attr && strlen(attr) > 0) {
    std::string frame;
    std::stringstream s(attr);
    s >> frame;
    // This could cause problems with the default pose assumptions in
    // the sdf parser, which I simply had to guess.
    DRAKE_ASSERT(0 && "this has not been tested yet");
    Eigen::Isometry3d T_frame =
        pose_map.at(frame);  // will throw an exception if the frame is not
                             // found.  that is the desired behavior.
    T = T_frame * T;
  } else {
    T = T_default_frame * T;
  }
}

namespace {
// Searches in directory @p path for a file called "package.xml".
// Adds the package name specified in package.xml and the path to the
// package to @p package_map.
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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

      // Skips hidden directories (including, importantly, "." and "..").
      if (file.is_dir && (file.name[0] != '.')) {
        searchDirectory(package_map, file.path);
      } else if (file.name == target_filename) {
        // Parses the package.xml file to find the name of the package.
        std::string package_name;

        {
          std::string file_name = std::string(file.path);

          XMLDocument xml_doc;
          xml_doc.LoadFile(file_name.data());
          if (xml_doc.ErrorID()) {
            throw std::runtime_error(
                "xmlUtil.cpp: searchDirectory: Failed to parse XML in file " +
                file_name + "\n" + xml_doc.ErrorName());
          }

          XMLElement* package_node = xml_doc.FirstChildElement("package");
          if (!package_node) {
            throw std::runtime_error(
                "xmlUtil.cpp: searchDirectory: ERROR: XML file \"" + file_name +
                "\" does not contain a <package> element.");
          }

          XMLElement* name_node = package_node->FirstChildElement("name");
          if (!name_node) {
            throw std::runtime_error(
                "xmlUtil.cpp: searchDirectory: ERROR: <package> element does "
                "not contain a <name> element. (XML file \"" +
                file_name + "\")");
          }

          package_name = name_node->FirstChild()->Value();
        }

        spruce::path mypath_s(file.path);

        // Don't overwrite entries in the map.
        auto package_iter = package_map.find(package_name);
        if (package_iter == package_map.end()) {
          package_map.insert(
              make_pair(package_name, mypath_s.root().append("/")));
        } else {
          std::cerr << "xmlUtil.cpp: searchDirectory: WARNING: Package \""
                    << package_name
                    << "\" was found more than once in the search "
                    << "space." << std::endl;
        }
      }
      tinydir_next(&dir);
    }
    tinydir_close(&dir);
  }
}
}  // anonymous namespace

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void populatePackageMap(map<string, string>& package_map) {
  // Since Drake's package.xml file is located at its super-build level,
  // remove the last "drake" directory from the drake path. Also omit the
  // trailing slash.
  const std::string drake_path = drake::GetDrakePath();
  const std::string drake_path_parent = drake_path.substr(
      0, drake_path.find_last_of("drake") - std::string("drake").size());

  searchDirectory(package_map, drake_path_parent);

  char* cstrpath = getenv("ROS_ROOT");
  if (cstrpath) searchDirectory(package_map, cstrpath);

  cstrpath = getenv("ROS_PACKAGE_PATH");
  if (cstrpath) searchDirectory(package_map, cstrpath);
}

namespace {
bool rospack(const string& package, const map<string, string>& package_map,
             // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
}  // anonymous namespace

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
      for (int i = 1; i < static_cast<int>(split_raw.size()) - 2; ++i) {
        mesh_filename_s.append(split_raw.at(i + 2));
      }
    } else {
      cerr << "Warning: Mesh '" << filename
           << "' could not be resolved and will be ignored by Drake." << endl;
      return string();
    }
  } else {
    std::string normalized_root_dir = spruce::path(root_dir).getStr();

// if root_dir is a relative path then convert it to absolute
#ifdef _WIN32
    bool dirIsRelative = !(normalized_root_dir.size() >= 2 &&
                           std::isalpha(normalized_root_dir[0]) &&
                           normalized_root_dir[1] == ':');
#else
    bool dirIsRelative =
        !(normalized_root_dir.size() >= 1 && normalized_root_dir[0] == '/');
#endif
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
    cerr << "Warning: File '" << mesh_filename_s.getStr()
         << "' could not be found." << endl;
    cerr << "Warning: Mesh '" << filename
         << "' could not be resolved and will be ignored by Drake." << endl;
    return string();
  }
  return mesh_filename_s.getStr();
}
