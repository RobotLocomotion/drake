#include "drake/multibody/parsing/package_map.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <initializer_list>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <tuple>
#include <utility>

#include <drake_vendor/tinyxml2.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {

namespace fs = std::filesystem;

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

namespace {

/* PackageData encapsulates everything we know about an added package:
 - The settings that were given to us when it was added.
 - A deprecation status that can be added after construction. */
class PackageData {
 public:
  /* Declares a local package (with a local filesystem path). */
  static PackageData MakeLocal(std::string path) {
    PackageData result;
    result.path_ = std::move(path);
    return result;
  }

  // Supports copy and move.
  PackageData(const PackageData&) = default;
  PackageData(PackageData&&) = default;

  // Does not support assignment.
  void operator=(const PackageData&) = delete;

  /* Returns true iff this package is deprecated. */
  bool is_deprecated() const { return deprecation_.has_value(); }

  /* Returns the (non-empty) deprecation message for this package, or nullopt
  when it's not deprecated. */
  const std::optional<std::string> get_deprecation() const {
    return deprecation_;
  }

  /* Sets the deprecation status and message for this package; if deprecated
  using an empty message, a default message will be provided instead. */
  void set_deprecation(std::optional<std::string> new_value) {
    if (new_value.has_value() && new_value->empty()) {
      deprecation_ = "(no explanation given)";
    } else {
      deprecation_ = std::move(new_value);
    }
  }

  /* Returns the path suitable for use in error messages. */
  const std::string& display_path() const { return path_; }

  /* Returns true iff `other` specifies a suitably identical pacakge to `this`
  such that we can fold the two together: the path specification
  must match, but the deprecation or fetch status can differ. In
  case they do not match, the error (if provided) will be reset to describe
  the incompatibility. */
  bool CanMerge(const PackageData& other, std::string* error = nullptr) const;

  /* Merges `other` into `this`. Throws an exception if CanMerge() is false.
  The `package_name` is non-functional (only used when reporting errors). */
  void Merge(std::string_view package_name, const PackageData& other);

  /* Returns the local filesystem path for this package.
  TODO(jwnimmer-tri) We should fetch remote packages here. */
  const std::string& GetPath() const { return path_; }

 private:
  PackageData() = default;

  /* Directory in which the manifest resides. */
  std::string path_;

  /* Optional message declaring deprecation of the package. If it is non-nullopt
  then it is also guaranteed to be non-empty. */
  std::optional<std::string> deprecation_;
};

}  // namespace

struct PackageMap::Impl {
  /* Adds the given `package_name, data` pair to the map while guarding for
  duplicates. Throws an exception if the package_name has already been added
  with different details. */
  void Emplace(std::string package_name, PackageData data) {
    // Reject packages with the same name but different details.
    auto iter = map.find(package_name);
    if (iter != map.end()) {
      std::string error;
      if (!iter->second.CanMerge(data, &error)) {
        throw std::runtime_error(fmt::format(
            "PackageMap::Add() cannot add '{}' {}", package_name, error));
      }
      // The package already exists; since the new package cannot have a
      // deprecation annotation yet, there's nothing more to do.
      DRAKE_DEMAND(data.is_deprecated() == false);
      return;
    }

    // Everything checks out, so we can add it now.
    map.emplace_hint(iter, std::move(package_name), std::move(data));
  }

  /* The key is the name of a ROS package and the value is a struct containing
  information about that package. */
  std::map<std::string, PackageData> map;
};

PackageMap::PackageMap(std::nullopt_t) : impl_{std::make_unique<Impl>()} {}

PackageMap::PackageMap(const PackageMap& other)
    : impl_{std::make_unique<Impl>(*other.impl_)} {}

PackageMap::PackageMap(PackageMap&& other)
    : impl_{std::exchange(other.impl_, std::make_unique<Impl>())} {}

PackageMap& PackageMap::operator=(const PackageMap& other) {
  return *this = PackageMap(other);
}

PackageMap& PackageMap::operator=(PackageMap&& other) {
  impl_ = std::exchange(other.impl_, std::make_unique<Impl>());
  return *this;
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

PackageMap::~PackageMap() = default;

PackageMap PackageMap::MakeEmpty() {
  return PackageMap(std::nullopt);
}

void PackageMap::Add(const std::string& package_name,
                     const std::string& package_path) {
  drake::log()->trace("PackageMap.Add('{}', '{}')", package_name, package_path);

  // Reject missing directories.
  if (!fs::is_directory(package_path)) {
    throw std::runtime_error(fmt::format(
        "PackageMap::Add cannot add '{}' because directory '{}' does not exist",
        package_name, package_path));
  }

  // Add it now. Emplace will handle rejection of duplicates.
  impl_->Emplace(package_name, PackageData::MakeLocal(package_path));
}

bool PackageData::CanMerge(const PackageData& other, std::string* error) const {
  // The paths must resolve to the same dir.
  if (!fs::equivalent(this->path_, other.path_)) {
    if (error != nullptr) {
      *error = fmt::format(
          "because the local paths are not equivalent ('{}' vs '{}')",
          this->display_path(), other.display_path());
    }
    return false;
  }
  return true;
}

void PackageData::Merge(std::string_view package_name,
                        const PackageData& other) {
  // Check whether `this` and `other` specify equivalent paths.
  std::string error;
  if (!CanMerge(other, &error)) {
    throw std::runtime_error(fmt::format(
        "PackageMap::AddMap cannot merge the package definition for '{}' {}",
        package_name, error));
  }

  // The paths are equivalent. The only thing remaining to merge is the
  // deprecation status. For that data, we'll take it from either party.
  if (!this->is_deprecated() && other.is_deprecated()) {
    set_deprecation(other.get_deprecation());
  }
}

void PackageMap::AddMap(const PackageMap& other_map) {
  for (const auto& [package_name, data] : other_map.impl_->map) {
    auto iter = impl_->map.find(package_name);
    if (iter == impl_->map.end()) {
      impl_->map.emplace_hint(iter, package_name, data);
    } else {
      iter->second.Merge(package_name, data);
    }
  }
}

bool PackageMap::Contains(const std::string& package_name) const {
  return impl_->map.find(package_name) != impl_->map.end();
}

void PackageMap::Remove(const std::string& package_name) {
  if (impl_->map.erase(package_name) == 0) {
    throw std::runtime_error(fmt::format(
        "Could not find and remove package://{} from the search path.",
        package_name));
  }
}

void PackageMap::SetDeprecated(const std::string& package_name,
                               std::optional<std::string> deprecated_message) {
  DRAKE_THROW_UNLESS(Contains(package_name));
  impl_->map.at(package_name).set_deprecation(std::move(deprecated_message));
}

int PackageMap::size() const {
  return impl_->map.size();
}

std::optional<std::string> PackageMap::GetDeprecated(
    const std::string& package_name) const {
  DRAKE_THROW_UNLESS(Contains(package_name));
  return impl_->map.at(package_name).get_deprecation();
}

std::vector<std::string> PackageMap::GetPackageNames() const {
  std::vector<std::string> package_names;
  package_names.reserve(impl_->map.size());
  for (const auto& [package_name, _] : impl_->map) {
    package_names.push_back(package_name);
  }
  return package_names;
}

const std::string& PackageMap::GetPath(
    const std::string& package_name,
    std::optional<std::string>* deprecated_message) const {
  DRAKE_THROW_UNLESS(Contains(package_name));
  const auto& data = impl_->map.at(package_name);

  // Check if we need to produce a deprecation warning.
  std::optional<std::string> warning;
  if (data.is_deprecated()) {
    warning = fmt::format("Package '{}' is deprecated: {}", package_name,
                          *data.get_deprecation());
  }

  // Copy the warning to the output parameter, or else the logger.
  if (deprecated_message != nullptr) {
    *deprecated_message = std::move(warning);
  } else if (warning.has_value()) {
    drake::log()->warn("PackageMap: {}", *warning);
  }

  return data.GetPath();
}

void PackageMap::PopulateFromFolder(const std::string& path) {
  DRAKE_THROW_UNLESS(!path.empty());
  CrawlForPackages(path);
}

void PackageMap::PopulateFromEnvironment(
    const std::string& environment_variable) {
  DRAKE_THROW_UNLESS(!environment_variable.empty());
  if (environment_variable == "ROS_PACKAGE_PATH") {
    throw std::logic_error(
        "PackageMap::PopulateFromEnvironment() must not be used to load a "
        "\"ROS_PACKAGE_PATH\"; use PopulateFromRosPackagePath() instead.");
  }
  const char* const value = std::getenv(environment_variable.c_str());
  if (value == nullptr) {
    return;
  }
  std::istringstream iss{std::string(value)};
  std::string path;
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
  std::istringstream input{std::string(value)};
  std::string path;
  while (std::getline(input, path, ':')) {
    if (!path.empty()) {
      CrawlForPackages(path, true, stop_markers);
    }
  }
}

namespace {

// Returns the parent directory of @p directory.
std::string GetParentDirectory(const std::string& directory) {
  DRAKE_DEMAND(!directory.empty());
  return fs::path(directory).parent_path().string();
}

// Removes leading and trailing whitespace and line breaks from a string.
std::string RemoveBreaksAndIndentation(std::string target) {
  static const never_destroyed<std::regex> midspan_breaks("\\s*\\n\\s*");
  static const never_destroyed<std::string> whitespace_characters(" \r\n\t");
  target.erase(0, target.find_first_not_of(whitespace_characters.access()));
  target.erase(target.find_last_not_of(whitespace_characters.access()) + 1);
  return std::regex_replace(target, midspan_breaks.access(), " ");
}

// Parses the package.xml file specified by package_xml_file. Finds and returns
// the name of the package and an optional deprecation message.
std::tuple<std::string, std::optional<std::string>> ParsePackageManifest(
    const std::string& package_xml_file) {
  DRAKE_DEMAND(!package_xml_file.empty());
  XMLDocument xml_doc;
  xml_doc.LoadFile(package_xml_file.data());
  if (xml_doc.ErrorID()) {
    throw std::runtime_error(fmt::format(
        "PackageMap::GetPackageName(): Failed to parse XML in file \"{}\"."
        "\n{}",
        package_xml_file, xml_doc.ErrorName()));
  }

  XMLElement* package_node = xml_doc.FirstChildElement("package");
  if (!package_node) {
    throw std::runtime_error(fmt::format(
        "PackageMap::GetPackageName(): ERROR: XML file \"{}\" does not "
        "contain element <package>.",
        package_xml_file));
  }

  XMLElement* name_node = package_node->FirstChildElement("name");
  if (!name_node) {
    throw std::runtime_error(fmt::format(
        "PackageMap::GetPackageName(): ERROR: <package> element does not "
        "contain element <name> (XML file \"{}\").",
        package_xml_file));
  }

  // Throws an exception if the name node does not have any children.
  DRAKE_THROW_UNLESS(!name_node->NoChildren());
  const std::string package_name = name_node->FirstChild()->Value();
  DRAKE_THROW_UNLESS(package_name != "");

  std::optional<std::string> deprecated_message;
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

void PackageMap::CrawlForPackages(
    const std::string& path, bool stop_at_package,
    const std::vector<std::string_view>& stop_markers) {
  DRAKE_THROW_UNLESS(!path.empty());
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
    const std::string package_path = dir.string() + "/";
    if (!Contains(package_name)) {
      Add(package_name, package_path);
      SetDeprecated(package_name, deprecated_message);
    } else {
      // Warn if we've found the same path with a different spelling.
      const PackageData& existing_data = impl_->map.at(package_name);
      if (!existing_data.CanMerge(PackageData::MakeLocal(package_path))) {
        drake::log()->warn(
            "PackageMap is ignoring newly-found path '{}' for package '{}'"
            " and will continue using the previously-known path at '{}'.",
            path, package_name, existing_data.display_path());
      }
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
      const std::string filename = entry.path().filename().string();
      // Skips hidden directories (including "." and "..").
      if (filename.at(0) == '.') {
        continue;
      }
      CrawlForPackages(entry.path().string(), stop_at_package, stop_markers);
    }
  }
}

void PackageMap::AddPackageXml(const std::string& filename) {
  DRAKE_THROW_UNLESS(!filename.empty());
  const auto [package_name, deprecated_message] =
      ParsePackageManifest(filename);
  const std::string package_path = GetParentDirectory(filename);
  Add(package_name, package_path);
  SetDeprecated(package_name, deprecated_message);
}

std::ostream& operator<<(std::ostream& out, const PackageMap& package_map) {
  out << "PackageMap:\n";
  if (package_map.size() == 0) {
    out << "  [EMPTY!]\n";
  }
  for (const auto& [package_name, data] : package_map.impl_->map) {
    out << "  - " << package_name << ": " << data.display_path() << "\n";
  }
  return out;
}

}  // namespace multibody
}  // namespace drake
