#include "drake/multibody/parsing/package_map.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <regex>
#include <sstream>
#include <tuple>
#include <utility>

#include <drake_vendor/tinyxml2.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

namespace drake {
namespace multibody {

namespace fs = std::filesystem;

using std::runtime_error;
using std::string;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

PackageMap::PackageMap(std::nullopt_t) {
  // Any common initialization code for all constructors could go here.
}

PackageMap::PackageMap() : PackageMap{std::nullopt} {
  // FindResource is the source of truth for where Drake's first-party files
  // live, no matter whether we're building from source or using a installed
  // version of Drake.
  const std::string drake_package = FindResourceOrThrow("drake/package.xml");
  AddPackageXml(drake_package);

  // For drake_models (i.e., https://github.com/RobotLocomotion/models), we need
  // to do something different for source vs installed, hinging on whether or
  // nor we have runfiles. When running from a uninstalled program, we have
  // bazel runfiles and will that source for drake_models. Otherwise, we're
  // using installed Drake, in which case the drake_models package is a sibling
  // to the drake package.
  if (HasRunfiles()) {
    // This is the case for Bazel-aware programs or tests, either first-party
    // use of Bazel in Drake, or also for downstream Bazel projects that are
    // using Drake as a dependency.
    const RlocationOrError find = FindRunfile("models_internal/package.xml");
    if (!find.error.empty()) {
      throw std::runtime_error(fmt::format(
          "PackageMap: Could not locate drake_models/package.xml in runfiles "
          "at @models_internal//:package.xml: {}",
          find.error));
    }
    AddPackageXml(find.abspath);
  } else {
    // This is the case for installed Drake. The models are installed under
    //  $prefix/share/drake_models/package.xml
    // which is a sibling to
    //  $prefix/share/drake/package.xml
    auto share_drake = std::filesystem::path(drake_package).parent_path();
    auto share = share_drake.parent_path().lexically_normal();
    AddPackageXml((share / "drake_models/package.xml").string());
  }
}

PackageMap PackageMap::MakeEmpty() {
  return PackageMap(std::nullopt);
}

void PackageMap::Add(const string& package_name, const string& package_path) {
  if (!AddPackageIfNew(package_name, package_path)) {
    throw std::runtime_error(fmt::format(
        "PackageMap already contains package \"{}\" with path \"{}\" that "
        "conflicts with provided path \"{}\"",
        package_name, map_.at(package_name).path, package_path));
  }
}

void PackageMap::AddMap(const PackageMap& other_map) {
  for (const auto& [package_name, data] : other_map.map_) {
    Add(package_name, data.path);
    SetDeprecated(package_name, data.deprecated_message);
  }
}

bool PackageMap::Contains(const string& package_name) const {
  return map_.find(package_name) != map_.end();
}

void PackageMap::Remove(const string& package_name) {
  if (map_.erase(package_name) == 0) {
    throw std::runtime_error(fmt::format(
        "Could not find and remove package://{} from the search path.",
        package_name));
  }
}

void PackageMap::SetDeprecated(const std::string& package_name,
                               std::optional<std::string> deprecated_message) {
  DRAKE_DEMAND(Contains(package_name));
  map_.at(package_name).deprecated_message = std::move(deprecated_message);
}

int PackageMap::size() const {
  return map_.size();
}

std::optional<std::string> PackageMap::GetDeprecated(
    const std::string& package_name) const {
  DRAKE_DEMAND(Contains(package_name));
  return map_.at(package_name).deprecated_message;
}

std::vector<std::string> PackageMap::GetPackageNames() const {
  std::vector<std::string> package_names;
  package_names.reserve(map_.size());
  for (const auto& [package_name, data] : map_) {
    unused(data);
    package_names.push_back(package_name);
  }
  return package_names;
}

const string& PackageMap::GetPath(
    const string& package_name,
    std::optional<std::string>* deprecated_message) const {
  DRAKE_DEMAND(Contains(package_name));
  const auto& package_data = map_.at(package_name);

  // Check if we need to produce a deprecation warning.
  std::optional<string> warning;
  if (package_data.deprecated_message.has_value()) {
    if (package_data.deprecated_message->empty()) {
      warning = fmt::format("Package \"{}\" is deprecated.", package_name);
    } else {
      warning = fmt::format("Package \"{}\" is deprecated: {}", package_name,
                            *package_data.deprecated_message);
    }
  }

  // Copy the warning to the output parameter, or else the logger.
  if (deprecated_message != nullptr) {
    *deprecated_message = warning;
  } else if (warning.has_value()) {
    drake::log()->warn("PackageMap: {}", *warning);
  }

  return package_data.path;
}

void PackageMap::PopulateFromFolder(const string& path) {
  DRAKE_DEMAND(!path.empty());
  CrawlForPackages(path);
}

void PackageMap::PopulateFromEnvironment(const string& environment_variable) {
  DRAKE_DEMAND(!environment_variable.empty());
  if (environment_variable == "ROS_PACKAGE_PATH") {
    throw std::logic_error(
        "PackageMap::PopulateFromEnvironment() must not be used to load a "
        "\"ROS_PACKAGE_PATH\"; use PopulateFromRosPackagePath() instead.");
  }
  const char* const value = std::getenv(environment_variable.c_str());
  if (value == nullptr) {
    return;
  }
  std::istringstream iss{string(value)};
  string path;
  while (std::getline(iss, path, ':')) {
    if (!path.empty()) {
      CrawlForPackages(path);
    }
  }
}

void PackageMap::PopulateFromRosPackagePath() {
  const std::vector<std::string_view> stop_markers = {
      "AMENT_IGNORE",
      "CATKIN_IGNORE",
      "COLCON_IGNORE",
  };

  const char* const value = std::getenv("ROS_PACKAGE_PATH");
  if (value == nullptr) {
    return;
  }
  std::istringstream input{string(value)};
  string path;
  while (std::getline(input, path, ':')) {
    if (!path.empty()) {
      CrawlForPackages(path, true, stop_markers);
    }
  }
}

namespace {

// Returns the parent directory of @p directory.
string GetParentDirectory(const string& directory) {
  DRAKE_DEMAND(!directory.empty());
  return fs::path(directory).parent_path().string();
}

// Removes leading and trailing whitespace and line breaks from a string.
string RemoveBreaksAndIndentation(string target) {
  static const std::regex midspan_breaks("\\s*\\n\\s*");
  static const string whitespace_characters = " \r\n\t";
  target.erase(0, target.find_first_not_of(whitespace_characters));
  target.erase(target.find_last_not_of(whitespace_characters) + 1);
  return std::regex_replace(target, midspan_breaks, " ");
}

// Parses the package.xml file specified by package_xml_file. Finds and returns
// the name of the package and an optional deprecation message.
std::tuple<string, std::optional<string>> ParsePackageManifest(
    const string& package_xml_file) {
  DRAKE_DEMAND(!package_xml_file.empty());
  XMLDocument xml_doc;
  xml_doc.LoadFile(package_xml_file.data());
  if (xml_doc.ErrorID()) {
    throw runtime_error(fmt::format(
        "PackageMap::GetPackageName(): Failed to parse XML in file \"{}\"."
        "\n{}",
        package_xml_file, xml_doc.ErrorName()));
  }

  XMLElement* package_node = xml_doc.FirstChildElement("package");
  if (!package_node) {
    throw runtime_error(fmt::format(
        "PackageMap::GetPackageName(): ERROR: XML file \"{}\" does not "
        "contain element <package>.",
        package_xml_file));
  }

  XMLElement* name_node = package_node->FirstChildElement("name");
  if (!name_node) {
    throw runtime_error(fmt::format(
        "PackageMap::GetPackageName(): ERROR: <package> element does not "
        "contain element <name> (XML file \"{}\").",
        package_xml_file));
  }

  // Throws an exception if the name node does not have any children.
  DRAKE_THROW_UNLESS(!name_node->NoChildren());
  const string package_name = name_node->FirstChild()->Value();
  DRAKE_THROW_UNLESS(package_name != "");

  std::optional<string> deprecated_message;
  XMLElement* export_node = package_node->FirstChildElement("export");
  if (export_node) {
    XMLElement* deprecated_node = export_node->FirstChildElement("deprecated");
    if (deprecated_node) {
      if (deprecated_node->NoChildren()) {
        deprecated_message.emplace("");
      } else {
        deprecated_message =
            RemoveBreaksAndIndentation(deprecated_node->FirstChild()->Value());
      }
    }
  }

  return {package_name, deprecated_message};
}

}  // namespace

bool PackageMap::AddPackageIfNew(const string& package_name,
                                 const string& path) {
  DRAKE_DEMAND(!package_name.empty());
  DRAKE_DEMAND(!path.empty());
  // Don't overwrite entries in the map.
  if (!Contains(package_name)) {
    drake::log()->trace("PackageMap: Adding package://{}: {}", package_name,
                        path);
    if (!fs::is_directory(path)) {
      throw std::runtime_error(fmt::format(
          "Could not add package://{} to the search path because directory {} "
          "does not exist",
          package_name, path));
    }
    map_.insert(make_pair(package_name, PackageData{path}));
  } else {
    // Don't warn if we've found the same path with a different spelling.
    const PackageData existing_data = map_.at(package_name);
    if (!fs::equivalent(existing_data.path, path)) {
      drake::log()->warn(
          "PackageMap is ignoring newly-found path \"{}\" for package \"{}\""
          " and will continue using the previously-known path at \"{}\".",
          path, package_name, existing_data.path);
      return false;
    }
  }
  return true;
}

void PackageMap::CrawlForPackages(
    const string& path, bool stop_at_package,
    const std::vector<std::string_view>& stop_markers) {
  DRAKE_DEMAND(!path.empty());
  fs::path dir = fs::path(path).lexically_normal();
  if (std::any_of(stop_markers.begin(), stop_markers.end(),
                  [dir](std::string_view name) {
                    return fs::exists(dir / name);
                  })) {
    return;
  }
  fs::path manifest = dir / "package.xml";
  if (fs::exists(manifest)) {
    const auto [package_name, deprecated_message] =
        ParsePackageManifest(manifest.string());
    const string package_path = dir.string();
    if (AddPackageIfNew(package_name, package_path + "/")) {
      SetDeprecated(package_name, deprecated_message);
    }
    if (stop_at_package) {
      return;
    }
  }
  std::error_code ec;
  fs::directory_iterator iter(dir, ec);
  if (ec) {
    log()->warn("Unable to open directory: {}", path);
    return;
  }
  for (const auto& entry : iter) {
    if (entry.is_directory()) {
      const string filename = entry.path().filename().string();
      // Skips hidden directories (including "." and "..").
      if (filename.at(0) == '.') {
        continue;
      }
      CrawlForPackages(entry.path().string(), stop_at_package, stop_markers);
    }
  }
}

void PackageMap::AddPackageXml(const string& filename) {
  const auto [package_name, deprecated_message] =
      ParsePackageManifest(filename);
  const string package_path = GetParentDirectory(filename);
  Add(package_name, package_path);
  SetDeprecated(package_name, deprecated_message);
}

std::ostream& operator<<(std::ostream& out, const PackageMap& package_map) {
  out << "PackageMap:\n";
  if (package_map.size() == 0) {
    out << "  [EMPTY!]\n";
  }
  for (const auto& entry : package_map.map_) {
    out << "  - " << entry.first << ": " << entry.second.path << "\n";
  }
  return out;
}

}  // namespace multibody
}  // namespace drake
