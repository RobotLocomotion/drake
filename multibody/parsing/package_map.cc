#include "drake/multibody/parsing/package_map.h"

#include <algorithm>
#include <cctype>
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
#include "drake/common/drake_throw.h"
#include "drake/common/find_cache.h"
#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/common/yaml/yaml_io.h"

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
  // not we have runfiles. When running from source, we have bazel runfiles and
  // will find our drake_models there. Otherwise, we're using installed Drake,
  // in which case the drake_models package is a sibling to the drake package.
  if (HasRunfiles()) {
    // This is the case for Bazel-aware programs or tests, either first-party
    // use of Bazel in Drake, or also for downstream Bazel projects that are
    // using Drake as a dependency.
    const RlocationOrError find = FindRunfile("models_internal/package.xml");
    DRAKE_DEMAND(find.error.empty());
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
    if (data.is_fetched()) {
      Add(package_name, data.path);
      SetDeprecated(package_name, data.deprecated_message);
    } else {
      DRAKE_DEMAND(data.remote.has_value());
      AddRemotePackage(package_name, *data.remote);
    }
  }
}

void PackageMap::AddRemotePackage(std::string package_name,
                                  RemotePackageParams params) {
  // Validate our arguments.
  auto iter = map_.find(package_name);
  if (iter != map_.end()) {
    // Adding a 100% identical package is supported (and a no-op).
    // Otherwise, it's an error.
    if (iter->second.remote.has_value()) {
      const RemotePackageParams& old_params = iter->second.remote.value();
      const std::string old_json = yaml::SaveJsonString(old_params);
      const std::string new_json = yaml::SaveJsonString(params);
      if (new_json == old_json) {
        drake::log()->trace("AddRemotePackage skipping duplicate '{}'",
                            package_name);
        return;
      }
    }
    throw std::logic_error(fmt::format(
        "PackageMap::AddRemotePackage cannot add '{}' because a package of "
        "that name has already been registered",
        package_name));
  }
  if (params.urls.empty()) {
    throw std::logic_error(fmt::format(
        "PackageMap::AddRemotePackage on '{}' requires at least one URL",
        package_name));
  }
  for (const std::string_view url : params.urls) {
    if (!((url.substr(0, 8) == "https://") || (url.substr(0, 7) == "http://") ||
          (url.substr(0, 7) == "file://"))) {
      throw std::logic_error(fmt::format(
          "PackageMap::AddRemotePackage on '{}' used an unsupported URL '{}'",
          package_name, url));
    }
  }
  if (!((params.sha256.size() == 64) &&
        (std::all_of(params.sha256.begin(), params.sha256.end(), [](char ch) {
          return std::isxdigit(ch);
        })))) {
    throw std::logic_error(fmt::format(
        "PackageMap::AddRemotePackage on '{}' with invalid sha256 '{}'",
        package_name, params.sha256));
  }
  if (params.archive_type.has_value()) {
    const std::initializer_list<const char*> known_types = {
        "zip", "tar", "gztar", "bztar", "xztar"};
    if (std::count(known_types.begin(), known_types.end(),
                   *params.archive_type) == 0) {
      throw std::logic_error(fmt::format(
          "PackageMap::AddRemotePackage on '{}' has unsupported archive "
          "type '{}'",
          package_name, *params.archive_type));
    }
  }

  // Everything checks out, so we can add it now.
  PackageData data;
  data.remote = std::move(params);
  map_.emplace_hint(iter, std::move(package_name), std::move(data));
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

  // If this is a remote package and we haven't fetched it yet, do that now.
  if (!package_data.is_fetched()) {
    FetchContent(package_name, const_cast<PackageData*>(&package_data));
  }

  DRAKE_DEMAND(!package_data.path.empty());
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
    PackageData data;
    data.path = path;
    map_.insert(make_pair(package_name, data));
  } else {
    // Don't warn if we've found the same path with a different spelling.
    const PackageData existing_data = map_.at(package_name);
    if (!fs::equivalent(existing_data.path, path)) {
      drake::log()->warn(
          "PackageMap is ignoring newly-found path \"{}\" for package \"{}\""
          " and will continue using the previously-known path at \"{}\".",
          path, package_name, existing_data.printable_path());
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

namespace {
struct DownloaderArgs {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(package_name));
    remote.Serialize(a);
    a->Visit(DRAKE_NVP(output_dir));
  }
  std::string package_name;
  PackageMap::RemotePackageParams remote;
  std::string output_dir;
};
}  // namespace

void PackageMap::FetchContent(std::string_view package_name,
                              PackageData* data) {
  DRAKE_DEMAND(!package_name.empty());
  DRAKE_DEMAND(data != nullptr);
  DRAKE_DEMAND(!data->is_fetched());
  DRAKE_DEMAND(data->remote.has_value());
  DRAKE_DEMAND(data->path.empty());

  // Find and/or create the cache_dir.
  auto try_cache = internal::FindOrCreateCache("package_map");
  if (!try_cache.error.empty()) {
    throw std::runtime_error(fmt::format(
        "PackageMap: when downloading '{}', could not create temporary cache "
        "directory: {}",
        package_name, try_cache.error));
  }
  const fs::path cache_dir = std::move(try_cache.abspath);

  // See if the package has already been fetched.
  const fs::path package_dir = cache_dir / data->remote->sha256;
  std::error_code ec;
  if (fs::is_directory(package_dir, ec)) {
    data->path = package_dir.string();
    return;
  }

  // Write the downloader arguments to a JSON file.
  DownloaderArgs args{.package_name = std::string(package_name),
                      .remote = *data->remote,
                      .output_dir = package_dir.string()};
  std::string json_filename = fs::path(cache_dir / ".fetch_XXXXXX").string();
  const int temp_fd = ::mkstemp(json_filename.data());
  ::close(temp_fd);
  ScopeExit remove_yaml([&json_filename]() {
    fs::remove(json_filename);
  });
  yaml::SaveJsonFile(json_filename, args);

  // Shell out to the downloader to fetch the package.
  const std::string downloader =
      FindResourceOrThrow("drake/multibody/parsing/package_downloader.py");
  const std::string command =
      fmt::format("/usr/bin/python3 {} {}", downloader, json_filename);
  const int returncode = std::system(command.c_str());
  if (returncode != 0) {
    throw std::runtime_error(fmt::format(
        "PackageMap: when downloading '{}', the downloader experienced an "
        "error",
        package_name));
  }

  // Confirm that it actually fetched.
  if (!fs::is_directory(package_dir, ec)) {
    throw std::runtime_error(fmt::format(
        "PackageMap: when downloading '{}', the downloader claimed success but "
        "somehow did not actually download anything?!",
        package_name));
  }

  // Success
  data->path = package_dir.string();
}

std::string PackageMap::PackageData::printable_path() const {
  return !path.empty() ? path : remote.value().urls.front();
}

std::ostream& operator<<(std::ostream& out, const PackageMap& package_map) {
  out << "PackageMap:\n";
  if (package_map.size() == 0) {
    out << "  [EMPTY!]\n";
  }
  for (const auto& [package_name, data] : package_map.map_) {
    out << "  - " << package_name << ": " << data.printable_path() << "\n";
  }
  return out;
}

}  // namespace multibody
}  // namespace drake
