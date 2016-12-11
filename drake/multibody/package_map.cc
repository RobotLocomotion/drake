#include "drake/multibody/package_map.h"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <vector>

#include "spruce.hh"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging.h"
#include "drake/thirdParty/bsd/tinydir/tinydir.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

using std::cerr;
using std::endl;
using std::getenv;
using std::istringstream;
using std::runtime_error;
using std::string;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace drake {
namespace parsers {

PackageMap::PackageMap() {}

PackageMap::~PackageMap() {}

void PackageMap::Add(const string& package_name, const string& package_path) {
  DRAKE_DEMAND(map_.find(package_name) == map_.end());
  map_.insert(make_pair(package_name, package_path));
}

bool PackageMap::Contains(const string& package_name) const {
  return map_.find(package_name) != map_.end();
}

const string& PackageMap::GetPath(const string& package_name) const {
  DRAKE_DEMAND(Contains(package_name));
  return map_.at(package_name);
}

void PackageMap::PopulateFromFolder(const string& path) {
  CrawlForPackages(path);
}

void PackageMap::PopulateFromEnvironment(const string& environment_variable) {
  const char* path_char = getenv(environment_variable.c_str());
  DRAKE_DEMAND(path_char);
  string path = string(path_char);
  CrawlForPackages(path);
}

// Returns true if @p directory has a package.xml file.
bool HasPackageMap(const string& directory) {
  spruce::path spruce_path(directory);
  spruce_path.append("package.xml");
  return spruce_path.exists();
}

// Returns the parent directory of @p directory.
string GetParentDirectory(const string& directory) {
  spruce::path spruce_path(directory);
  std::vector<std::string> v = spruce_path.split();
  DRAKE_DEMAND(v.size() > 0);
  spruce::path result;
  for (size_t i = 0; i < v.size() - 1; ++i) {
    if (v.at(i) != "")
      result.append(v.at(i));
  }
  return result.getStr();
}

// Parses the package.xml file specified by package_map_file. Finds and returns
// the name of the package.
std::string GetPackageName(const string& package_map_file) {
  XMLDocument xml_doc;
  xml_doc.LoadFile(package_map_file.data());
  if (xml_doc.ErrorID()) {
    throw runtime_error("parser_common.cc: CrawlForPackages(): "
        "Failed to parse XML in file \"" + package_map_file + "\".\n" +
        xml_doc.ErrorName());
  }

  XMLElement* package_node = xml_doc.FirstChildElement("package");
  if (!package_node) {
    throw runtime_error("parser_common.cc: CrawlForPackages(): "
        "ERROR: XML file \"" + package_map_file + "\" does not contain "
        "element <package>.");
  }

  XMLElement* name_node = package_node->FirstChildElement("name");
  if (!name_node) {
    throw runtime_error("parser_common.cc: CrawlForPackages(): "
        "ERROR: <package> element does not contain element <name> "
        "(XML file \"" + package_map_file + "\").");
  }

  return name_node->FirstChild()->Value();
}

void PackageMap::AddPackageIfNew(const string& package_name,
    const string& path) {
  // Don't overwrite entries in the map.
  if (!Contains(package_name)) {
    Add(package_name, path);
  } else {
    drake::log()->warn("Package \"{}\" was found more than once in the "
                       "search space.", package_name);
  }
}

void PackageMap::PopulateUpstreamToDrakeDistroHelper(const string& directory) {
  if (HasPackageMap(directory)) {
    spruce::path spruce_path(directory);
    spruce_path.append("package.xml");
    const string package_name = GetPackageName(spruce_path.getStr());
    if (!Contains(package_name))
      AddPackageIfNew(package_name, directory);
  }
  if (directory != "/" && directory != GetDrakePath())
    PopulateUpstreamToDrakeDistroHelper(GetParentDirectory(directory));
}

void PackageMap::PopulateUpstreamToDrakeDistro(const string& model_file) {
  // Aborts if the file is not in drake-distro.
  if (model_file.find(GetDrakePath()) == std::string::npos) return;

  spruce::path spruce_path(model_file);

  // Verifies that the model file is to a URDF or SDF file.
  auto extension = spruce_path.extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  DRAKE_DEMAND(extension == ".urdf" || extension == ".sdf");
  PopulateUpstreamToDrakeDistroHelper(spruce_path.root());
}

// Searches in directory @p path for files called "package.xml".
// Adds the package name specified in package.xml and the path to the
// package to @p package_map.
void PackageMap::CrawlForPackages(const string& path) {
  string directory_path = path;

  // Removes all trailing "/" characters if any exist.
  if (directory_path.length() > 0) {
    string::iterator it = directory_path.end() - 1;
    while (*it == '/') {
      directory_path.erase(it);
      if (directory_path.length() > 0)
        it = directory_path.end() - 1;
    }
  }

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
      if (file.is_dir && (file.name[0] != '.')) {
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

}  // namespace parsers
}  // namespace drake
