#include "drake/multibody/parsers/package_map.h"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <utility>
#include <vector>

#include <spruce.hh>
#include <tinydir.h>
#include <tinyxml2.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

using std::cerr;
using std::endl;
using std::getenv;
using std::istringstream;
using std::runtime_error;
using std::string;
using std::vector;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace drake {
namespace parsers {

PackageMap::PackageMap() {}

void PackageMap::Add(const string& package_name, const string& package_path) {
  DRAKE_DEMAND(map_.find(package_name) == map_.end());
  if (!spruce::path(package_path).exists()) {
    throw std::runtime_error(
        "Could not add package://" + package_name + " to the search path "
        "because directory " + package_path + " does not exist");
  }
  map_.insert(make_pair(package_name, package_path));
}

bool PackageMap::Contains(const string& package_name) const {
  return map_.find(package_name) != map_.end();
}

int PackageMap::size() const {
  return map_.size();
}

const string& PackageMap::GetPath(const string& package_name) const {
  DRAKE_DEMAND(Contains(package_name));
  return map_.at(package_name);
}

void PackageMap::PopulateFromFolder(const string& path) {
  DRAKE_DEMAND(!path.empty());
  CrawlForPackages(path);
}

void PackageMap::PopulateFromEnvironment(const string& environment_variable) {
  DRAKE_DEMAND(!environment_variable.empty());
  const char* path_char = getenv(environment_variable.c_str());
  DRAKE_DEMAND(path_char);
  string path = string(path_char);
  CrawlForPackages(path);
}

namespace {

// Returns true if @p directory has a package.xml file.
bool HasPackageXmlFile(const string& directory) {
  DRAKE_DEMAND(!directory.empty());
  spruce::path spruce_path(directory);
  spruce_path.append("package.xml");
  return spruce_path.exists();
}

// Returns the parent directory of @p directory.
string GetParentDirectory(const string& directory) {
  DRAKE_DEMAND(!directory.empty());
  return spruce::path(directory).root();
}

// Parses the package.xml file specified by package_xml_file. Finds and returns
// the name of the package.
string GetPackageName(const string& package_xml_file) {
  DRAKE_DEMAND(!package_xml_file.empty());
  XMLDocument xml_doc;
  xml_doc.LoadFile(package_xml_file.data());
  if (xml_doc.ErrorID()) {
    throw runtime_error("package_map.cc: GetPackageName(): "
        "Failed to parse XML in file \"" + package_xml_file + "\".\n" +
        xml_doc.ErrorName());
  }

  XMLElement* package_node = xml_doc.FirstChildElement("package");
  if (!package_node) {
    throw runtime_error("package_map.cc: GetPackageName(): "
        "ERROR: XML file \"" + package_xml_file + "\" does not contain "
        "element <package>.");
  }

  XMLElement* name_node = package_node->FirstChildElement("name");
  if (!name_node) {
    throw runtime_error("package_map.cc: GetPackageName(): "
        "ERROR: <package> element does not contain element <name> "
        "(XML file \"" + package_xml_file + "\").");
  }

  // Throws an exception if the name node does not have any children.
  DRAKE_THROW_UNLESS(!name_node->NoChildren());
  const string package_name = name_node->FirstChild()->Value();
  DRAKE_THROW_UNLESS(package_name != "");
  return package_name;
}

}  // namespace

void PackageMap::AddPackageIfNew(const string& package_name,
    const string& path) {
  DRAKE_DEMAND(!package_name.empty());
  DRAKE_DEMAND(!path.empty());
  // Don't overwrite entries in the map.
  if (!Contains(package_name)) {
    Add(package_name, path);
  } else {
    drake::log()->warn("Package \"{}\" was found more than once in the "
                       "search space.", package_name);
  }
}

void PackageMap::PopulateUpstreamToDrakeHelper(
    const string& directory,
    const string& stop_at_directory) {
  DRAKE_DEMAND(!directory.empty());

  // If we've reached the top, then stop searching.
  if (directory.length() <= stop_at_directory.length()) {
    return;
  }

  // If there is a new package.xml file, then add it.
  if (HasPackageXmlFile(directory)) {
    spruce::path spruce_path(directory);
    spruce_path.append("package.xml");
    const string package_name = GetPackageName(spruce_path.getStr());
    AddPackageIfNew(package_name, directory);
  }

  // Continue searching in our parent directory.
  PopulateUpstreamToDrakeHelper(
      GetParentDirectory(directory), stop_at_directory);
}

void PackageMap::PopulateUpstreamToDrake(const string& model_file) {
  DRAKE_DEMAND(!model_file.empty());

  // Verify that the model_file names an URDF or SDF file.
  spruce::path spruce_path(model_file);
  string extension = spruce_path.extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  DRAKE_DEMAND(extension == ".urdf" || extension == ".sdf");
  const string model_dir = spruce_path.root();

  // Bail out if the model file is not part of Drake.
  const optional<string> maybe_drake_path = MaybeGetDrakePath();
  if (!maybe_drake_path) {
    return;
  }
  const string& drake_path = *maybe_drake_path;
  auto iter = std::mismatch(drake_path.begin(), drake_path.end(),
                            model_dir.begin());
  if (iter.first != drake_path.end()) {
    // The drake_path was not a prefix of model_dir.
    return;
  }

  // Search the directory containing the model_file and "upstream".
  PopulateUpstreamToDrakeHelper(spruce_path.root(), drake_path);
}

void PackageMap::CrawlForPackages(const string& path) {
  DRAKE_DEMAND(!path.empty());
  string directory_path = path;

  // Removes all trailing "/" characters if any exist.
  while (directory_path.length() > 0 && *(directory_path.end() - 1) == '/') {
    directory_path.erase(directory_path.end() - 1);
  }
  DRAKE_DEMAND(directory_path.length() > 0);

  istringstream iss(directory_path);
  const string target_filename("package.xml");
  const char pathsep = ':';

  string token;
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
      if (file.is_dir && (string(file.name) != "")
          && (file.name[0] != '.')) {
        CrawlForPackages(file.path);
      } else if (file.name == target_filename) {
        const string package_name = GetPackageName(file.path);
        spruce::path spruce_path(file.path);
        const string package_path = spruce_path.root().append("/");
        AddPackageIfNew(package_name, package_path);
      }
      tinydir_next(&dir);
    }
    tinydir_close(&dir);
  }
}

std::ostream& operator<<(std::ostream& out, const PackageMap& package_map) {
    out << "PackageMap:\n";
    if (package_map.size() == 0)
      out << "  [EMPTY!]\n";
    for (const auto& entry : package_map.map_) {
      out << "  - " << entry.first << ": " << entry.second << "\n";
    }
    return out;
}

}  // namespace parsers
}  // namespace drake
