#include "drake/multibody/package_map.h"

#include <cstdlib>
#include <sstream>

#include "spruce.hh"

#include "drake/common/drake_assert.h"
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

// Searches in directory @p path for files called "package.xml".
// Adds the package name specified in package.xml and the path to the
// package to @p package_map.
void PackageMap::CrawlForPackages(const string& path) {
  string token, t;
  string directory_path = path;

  // Removes trailing "/" if it exists.
  if (directory_path.length() > 0) {
    string::iterator it = directory_path.end() - 1;
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
        CrawlForPackages(file.path);
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
        if (!Contains(package_name)) {
          Add(package_name, mypath_s.root().append("/"));
        } else {
          drake::log()->error(
              "parser_common.cc: CrawlForPackages: WARNING: Package \"{}\" "
              "was found more than once in the search space.", package_name);
        }
      }
      tinydir_next(&dir);
    }
    tinydir_close(&dir);
  }
}

}  // namespace parsers
}  // namespace drake
