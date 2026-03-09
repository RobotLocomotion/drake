#include "drake/multibody/parsing/package_map.h"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <initializer_list>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <tuple>
#include <utility>

#include <tinyxml2.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_cache.h"
#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/network_policy.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/scope_exit.h"
#include "drake/common/sha256.h"
#include "drake/common/text_logging.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/detail_path_utils.h"

namespace drake {
namespace multibody {

namespace fs = std::filesystem;

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

std::string PackageMap::RemoteParams::ToJson() const {
  return yaml::SaveJsonString(*this);
}

bool operator==(const PackageMap::RemoteParams& left,
                const PackageMap::RemoteParams& right) {
  return left.ToJson() == right.ToJson();
}

namespace {

/* Provides a little encapsulation for a string `path` along with atomics to
guard shared access to the data. This helps us preserve shared-access invariants
and allows PackageData to use the default implementations of C++ special member
functions (i.e., copy & move). */
class PathWithMutex {
 public:
  static PathWithMutex MakeLocal(std::string path) {
    PathWithMutex result;
    result.path_ = std::move(path);
    return result;
  }

  static PathWithMutex MakeRemote() {
    PathWithMutex result;
    result.needs_fetch_ = true;
    return result;
  }

  /* Copying preserves the value, but produces a distinct atomic variable. */
  PathWithMutex(const PathWithMutex& other)
      : needs_fetch_(other.needs_fetch_.load()), path_(other.path_) {}

  // Does not support copy-assignment nor move-assignment.
  PathWithMutex& operator=(const PathWithMutex&) = delete;

  /* Returns true iff the package is both remote and unfetched. */
  bool needs_fetch() const { return needs_fetch_.load(); }

  /* Returns the package path, either a local path or the fetched remote path.
  @pre needs_fetch() is false */
  const std::string& get_without_fetching() const {
    DRAKE_DEMAND(!needs_fetch_);
    DRAKE_DEMAND(!path_.empty());
    return path_;
  }

  /* Returns a tuple (lock_guard, mutable_path) for our path mutex.
  This is required prior to calling set_fetched_path(). */
  [[nodiscard]] auto lock() const {
    return std::pair<std::lock_guard<std::mutex>, PathWithMutex*>(
        mutex_, const_cast<PathWithMutex*>(this));
  }

  /* Sets the path for a (currently-unfetched) path.
  @pre needs_fetch() is true.
  @pre either the lock() is currently held or the caller has exclusive
  access to `this` */
  void set_fetched_path(std::string path) {
    DRAKE_DEMAND(needs_fetch_);
    DRAKE_DEMAND(path_.empty());
    path_ = std::move(path);
    needs_fetch_.store(false);
  }

 private:
  PathWithMutex() = default;

  std::atomic<bool> needs_fetch_{false};
  // When needs_fetch_ is true, the mutex_ guards the path_.
  // When needs_fetch_ is false, the path_ can be read without a mutex.
  mutable std::mutex mutex_;
  std::string path_;
};

/* PackageData encapsulates everything we know about an added package:
 - The settings that were given to us when it was added.
 - A deprecation status that can be added after construction.
 - For remote packages, whether it's been fetched locally yet and how to fetch
   it when needed.
*/
class PackageData {
 public:
  static PackageData MakeLocal(std::string path) {
    return PackageData(PathWithMutex::MakeLocal(std::move(path)));
  }

  /* Declares a remote package (with URLs, etc). Does not fetch anything.
  @pre `params` has already been validated to be well-formed. */
  static PackageData MakeRemote(PackageMap::RemoteParams params) {
    PackageData result(PathWithMutex::MakeRemote());
    result.remote_params_ = std::move(params);
    return result;
  }

  // Supports copy and move.
  PackageData(const PackageData&) = default;
  PackageData(PackageData&&) = default;

  // Does not support copy-assignment nor move-assignment.
  void operator=(const PackageData&) = delete;

  /* Returns true iff this package was added using a local path, e.g.,
  Add() or AddPackageXml() or PopulateFrom... etc. */
  bool is_local() const { return !remote_params_.has_value(); }

  /* Returns the path for a local package.
  @pre is_local() is true. */
  const std::string& local_path() const {
    DRAKE_DEMAND(is_local());
    return path_.get_without_fetching();
  }

  /* Returns true iff this package was added using AddRemote(). */
  bool is_remote() const { return remote_params_.has_value(); }

  /* Returns the remote params.
  @pre is_remote() is true. */
  const PackageMap::RemoteParams& remote_params() const {
    return remote_params_.value();
  }

  /* Returns true iff this package is deprecated. */
  bool is_deprecated() const { return deprecation_.has_value(); }

  /* Returns the (non-empty) deprecation message for this package, or nullopt
  when it's not deprecated. */
  const std::optional<std::string> get_deprecation() const {
    return deprecation_;
  }

  /* Sets the deprecation status and message for this package; if deprecated
  using an empty message, a default message will be provided instead; setting
  to nullopt sets the status to "not deprecated". */
  void set_deprecation(std::optional<std::string> new_value) {
    if (new_value.has_value() && new_value->empty()) {
      deprecation_ = "(no explanation given)";
    } else {
      deprecation_ = std::move(new_value);
    }
  }

  /* Returns the path suitable for use in error messages. For local packages,
  returns the local path; for remote packages, returns the first URL. */
  const std::string& display_path() const {
    return remote_params_.has_value() ? remote_params_.value().urls.front()
                                      : local_path();
  }

  /* Returns true iff `other` specifies a suitably identical package to `this`
  such that we can fold the two together: the path specification (whether local
  or remote) must match, but the deprecation or fetch status can differ. In case
  they do not match, the error will be reset to describe the incompatibility. */
  bool CanMerge(const PackageData& other, std::string* error) const;

  /* Merges `other` into `this`. Throws an exception if CanMerge() is false.
  The `package_name` is non-functional (only used when reporting errors). */
  void Merge(std::string_view package_name, const PackageData& other);

  /* Returns the path relative to our cache dir where the package will live once
  it is fetched (or lives already, if it's already fetched).
  @pre is_remote() is true. */
  fs::path GetCacheRelativePath() const;

  /* Checks whether a remote package is already in the cache and if so updates
  this object so that GetPathWithAutomaticFetching() will return the already-
  cached path instead of fetching anything.
  @pre is_remote() is true. */
  bool FindInCache();

  /* Returns the local filesystem path for this package. In case this package
  is remote and not already fetched, this function will fetch before returning.
  The `package_name` is non-functional (only used when reporting errors). */
  const std::string& GetPathWithAutomaticFetching(
      std::string_view package_name) const;

 private:
  explicit PackageData(const PathWithMutex& path) : path_(path) {}

  /* Directory in which the manifest resides (for remote packages, this starts
  unset, i.e., needs_fetch() is true). Refer to PathWithMutex for details. */
  PathWithMutex path_;

  /* If this was added using AddRemote(), this will contain the details, even
  after the package has been fetched and is_remote has changed to 'false'. In
  other words, this is a record of what was declared, even if `path_` has since
  been changed to point to the downloaded copy.) */
  std::optional<PackageMap::RemoteParams> remote_params_;

  /* Optional message declaring deprecation of the package. If it is non-nullopt
  then it is also guaranteed to be non-empty. */
  std::optional<std::string> deprecation_;
};

/* Given a list of urls (i.e., the RemoteParams::urls), returns a copy of the
list that reflects the current network policy (i.e., by removing any that are
not allowed). */
std::vector<std::string> GetAllowedUrls(const std::vector<std::string>& urls) {
  std::vector<std::string> result(urls);
  if (drake::internal::IsNetworkingAllowed("package_map")) {
    return result;
  }
  auto should_deny = [](const auto& url) {
    const bool is_allowed = url.substr(0, 7) == "file://";
    if (!is_allowed) {
      log()->trace("PackageMap ignoring '{}' due to DRAKE_ALLOW_NETWORK", url);
    }
    return !is_allowed;
  };
  result.erase(std::remove_if(result.begin(), result.end(), should_deny),
               result.end());
  return result;
}

fs::path PackageData::GetCacheRelativePath() const {
  DRAKE_DEMAND(is_remote());
  const std::string hashed_strip_prefix =
      Sha256::Checksum(remote_params_->strip_prefix.value_or("")).to_string();
  return fmt::format("{}-{}", remote_params_->sha256, hashed_strip_prefix);
}

/* A little helper struct that gathers the args for package_downloader into a
single place to make it easy to convert to JSON. */
struct DownloaderArgs {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(package_name));
    remote_params.Serialize(a);
    a->Visit(DRAKE_NVP(output_dir));
  }
  std::string package_name;
  PackageMap::RemoteParams remote_params;
  std::string output_dir;
};

bool PackageData::FindInCache() {
  DRAKE_DEMAND(is_remote());
  if (!path_.needs_fetch()) {
    return true;
  }
  drake::internal::PathOrError try_cache =
      drake::internal::FindOrCreateCache("package_map");
  if (!try_cache.error.empty()) {
    return false;
  }
  const fs::path package_dir = try_cache.abspath / GetCacheRelativePath();
  std::error_code ec;
  if (!fs::is_directory(package_dir, ec)) {
    return false;
  }
  path_.set_fetched_path(package_dir.string());
  return true;
}

const std::string& PackageData::GetPathWithAutomaticFetching(
    std::string_view package_name) const {
  DRAKE_DEMAND(!package_name.empty());

  if (!path_.needs_fetch()) {
    return path_.get_without_fetching();
  }
  DRAKE_DEMAND(is_remote());

  // Note that we are a *const* member function, but we're about to start
  // modifying member data as we fetch the package. Everything from here on
  // needs to have exclusive access to this package's PackageData object.
  auto [guard, mutable_path] = path_.lock();

  // We need to do the "double-checked locking" pattern; the bool might have
  // changed value while we were waiting for the lock.
  if (!path_.needs_fetch()) {
    return path_.get_without_fetching();
  }

  // Find and/or create the cache_dir.
  drake::internal::PathOrError try_cache =
      drake::internal::FindOrCreateCache("package_map");
  if (!try_cache.error.empty()) {
    throw std::runtime_error(fmt::format(
        "PackageMap: when downloading '{}', could not create temporary cache "
        "directory: {}",
        package_name, try_cache.error));
  }
  const fs::path cache_dir = std::move(try_cache.abspath);

  // See if the package has already been fetched.
  const fs::path package_dir = cache_dir / GetCacheRelativePath();
  std::error_code ec;
  if (fs::is_directory(package_dir, ec)) {
    mutable_path->set_fetched_path(package_dir.string());
    return path_.get_without_fetching();
  }

  drake::log()->info("PackageMap: Downloading {}", display_path());

  // Write the downloader arguments to a JSON file.
  DownloaderArgs args{.package_name = std::string(package_name),
                      .remote_params = remote_params(),
                      .output_dir = package_dir.string()};
  args.remote_params.urls = GetAllowedUrls(args.remote_params.urls);
  std::string json_filename = fs::path(cache_dir / ".fetch_XXXXXX").string();
  {
    const int temp_fd = ::mkstemp(json_filename.data());
    ::close(temp_fd);
  }
  ScopeExit remove_json_file([&json_filename]() {
    fs::remove(json_filename);
  });
  yaml::SaveJsonFile(json_filename, args);

  // Prepare an output file for the download to write error messages into.
  std::string error_filename = fs::path(cache_dir / ".error_XXXXXX").string();
  {
    const int temp_fd = ::mkstemp(error_filename.data());
    ::close(temp_fd);
  }
  ScopeExit remove_error_file([&error_filename]() {
    fs::remove(error_filename);
  });

  // Shell out to the downloader to fetch the package.
  const std::string downloader =
      FindResourceOrThrow("drake/multibody/parsing/package_downloader.py");
  const std::string command =
      fmt::format("/usr/bin/python3 -E {} {} {} {}", downloader, json_filename,
                  error_filename, "--disable-drake-valgrind-tracing");
  const int returncode = std::system(command.c_str());
  if (returncode != 0) {
    // Try to read the error message text from the downloader.
    std::optional<std::string> error = ReadFile(error_filename);
    if (!error || error->empty()) {
      error = fmt::format("returncode == {}", returncode);
    }
    throw std::runtime_error(fmt::format(
        "PackageMap: when downloading '{}', the downloader experienced an "
        "error: {}",
        package_name, *error));
  }

  // Confirm that it actually fetched.
  if (!fs::is_directory(package_dir, ec)) {
    throw std::runtime_error(fmt::format(
        "PackageMap: when downloading '{}', the downloader claimed success but "
        "somehow did not actually download anything?!",
        package_name));
  }

  // Success!
  mutable_path->set_fetched_path(package_dir.string());
  return path_.get_without_fetching();
}

}  // namespace

class PackageMap::Impl {
 public:
  Impl() = default;

  // Supports copy and move.
  Impl(const Impl&) = default;
  Impl(Impl&&) = default;

  // Does not support copy-assignment nor move-assignment.
  void operator=(const Impl&) = delete;

  /* Read-only access to the underlying map. This allows the PackageMap's const
  member functions to do their job without forwarding them all to the Impl. */
  const std::map<std::string, PackageData>& map() const { return map_; }

  /* The Impl flavor of the outer class's PackageMap::AddMap(). */
  void AddMap(const Impl& other_map) {
    for (const auto& [package_name, data] : other_map.map_) {
      auto iter = map_.find(package_name);
      if (iter == map_.end()) {
        map_.emplace_hint(iter, package_name, data);
      } else {
        iter->second.Merge(package_name, data);
      }
    }
  }

  /* The Impl flavor of the outer class's PackageMap::Remove(). */
  void Remove(const std::string& package_name) {
    if (map_.erase(package_name) == 0) {
      throw std::runtime_error(fmt::format(
          "Could not find and remove package://{} from the search path.",
          package_name));
    }
  }

  /* The Impl flavor of the outer class's PackageMap::SetDeprecated().
  @pre The package_name exists in the map. */
  void SetDeprecated(const std::string& package_name,
                     std::optional<std::string> new_value) {
    map_.at(package_name).set_deprecation(std::move(new_value));
  }

  /* Adds the given `package_name, data` pair to the map while guarding for
  duplicates. Throws an exception if the package_name has already been added
  with different details. */
  void Emplace(std::string package_name, PackageData data) {
    // When deciding whether Emplace can be a no-op, we assume that `data` does
    // not contain any deprecation message yet (so that we don't need to worry
    // about retaining that information here).
    DRAKE_DEMAND(data.is_deprecated() == false);

    // Reject packages with the same name but different details.
    auto iter = map_.find(package_name);
    if (iter != map_.end()) {
      std::string error;
      if (!iter->second.CanMerge(data, &error)) {
        throw std::runtime_error(
            fmt::format("PackageMap::Add{}() cannot add '{}' {}",
                        data.is_remote() ? "Remote" : "", package_name, error));
      }
      return;
    }

    // The package_name had not been added yet, so add it now.
    map_.emplace_hint(iter, std::move(package_name), std::move(data));
  }

 private:
  /* The key is the name of a ROS package and the value is a struct containing
  information about that package. */
  std::map<std::string, PackageData> map_;
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

namespace {

/* The schema used by tools/workspace/github.bzl to save the repository details.
We only need a subset of the fields, so only those few are declared here. */
struct RepositoryMetadataSchema {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(urls));
    a->Visit(DRAKE_NVP(sha256));
    a->Visit(DRAKE_NVP(strip_prefix));
  }
  std::vector<std::string> urls;
  std::string sha256;
  std::string strip_prefix;
};

}  // namespace

namespace internal {

PackageMap::RemoteParams GetDrakeModelsRemoteParams() {
  const std::string json_filename =
      FindResourceOrThrow("drake/multibody/parsing/drake_models.json");
  const yaml::LoadYamlOptions options{
      // Ignore any data that we don't need.
      .allow_yaml_with_no_cpp = true,
      // Require all data that we do need.
      .allow_cpp_with_no_yaml = false,
  };
  auto json_contents = yaml::LoadYamlFile<RepositoryMetadataSchema>(
      json_filename, {}, {}, options);
  PackageMap::RemoteParams result;
  result.urls = std::move(json_contents.urls);
  result.sha256 = std::move(json_contents.sha256);
  result.strip_prefix = std::move(json_contents.strip_prefix);
  return result;
}

}  // namespace internal

PackageMap::PackageMap() : PackageMap{std::nullopt} {
  // FindResource is the source of truth for where Drake's first-party files
  // live, no matter whether we're building from source or using a installed
  // version of Drake.
  const std::string drake_package = FindResourceOrThrow("drake/package.xml");
  AddPackageXml(drake_package);

  // Prepare the params for fetching remote drake_models. We do this outside of
  // the if-else to ensure it receives test coverage under bazel (i.e., even if
  // we're never going to download anything).
  static const never_destroyed<RemoteParams> memoized_params(
      internal::GetDrakeModelsRemoteParams());

  // For drake_models (i.e., https://github.com/RobotLocomotion/models), the
  // location where we find the data will vary. If we have Bazel runfiles with
  // an appropriately-declared external, we'll use that. Otherwise, we'll fetch
  // the data on demand from the internet.
  const RlocationOrError maybe_models_package_xml =
      FindRunfile("drake_models/package.xml");
  if (maybe_models_package_xml.error.empty()) {
    AddPackageXml(maybe_models_package_xml.abspath);
  } else {
    AddRemote("drake_models", memoized_params.access());
  }
}

PackageMap::~PackageMap() = default;

PackageMap PackageMap::MakeEmpty() {
  return PackageMap(std::nullopt);
}

void PackageMap::Add(const std::string& package_name,
                     const fs::path& package_path) {
  this->Add(package_name, package_path.string());
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

void PackageMap::Add(const std::string& package_name,
                     const char* package_path) {
  DRAKE_THROW_UNLESS(package_path != nullptr);
  this->Add(package_name, std::string(package_path));
}

namespace {
// Returns true iff the two paths exist and their fs::canonical respellings
// are fs::equivalent. Returns false if there are any filesystem errors.
bool is_equivalent_canonical(const fs::path& first, const fs::path& second) {
  std::error_code ec;
  const fs::path first_canonical = fs::canonical(first, ec);
  if (ec) {
    return false;
  }
  const fs::path second_canonical = fs::canonical(second, ec);
  if (ec) {
    return false;
  }
  return fs::equivalent(first_canonical, second_canonical, ec);
}
}  // namespace

bool PackageData::CanMerge(const PackageData& other, std::string* error) const {
  DRAKE_DEMAND(error != nullptr);
  // When both packages are local, the paths must either resolve to the same dir
  // or else (in support of Bazel runfiles) the package.xml files must resolve
  // to the same file.
  if (this->is_local() && other.is_local()) {
    const fs::path this_path = fs::path(this->local_path());
    const fs::path other_path = fs::path(other.local_path());
    if (!is_equivalent_canonical(this_path, other_path) &&
        !is_equivalent_canonical(this_path / "package.xml",
                                 other_path / "package.xml")) {
      *error = fmt::format(
          "because the local paths are not equivalent ('{}' vs '{}')",
          this->display_path(), other.display_path());
      return false;
    }
    return true;
  }

  // When both packages are remote, the specification must match exactly.
  if (this->is_remote() && other.is_remote()) {
    if (!(this->remote_params() == other.remote_params())) {
      *error = fmt::format(
          "because the remote package parameters differ ({} vs {})",
          this->remote_params().ToJson(), other.remote_params().ToJson());
      return false;
    }
    return true;
  }

  // Cannot merge local paths with remote params.
  *error = fmt::format(
      "because the existing path is {} ('{}') "
      "but the the new path is {} ('{}')",
      this->is_local() ? "local" : "remote", this->display_path(),
      other.is_local() ? "local" : "remote", other.display_path());
  return false;
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

  // The equivalent paths now need to merge deprecation status. The result is
  // deprecated if this or other are deprecated, favoring the current message
  // when it exists.
  if (!this->is_deprecated() && other.is_deprecated()) {
    set_deprecation(other.get_deprecation());
  }
}

void PackageMap::AddMap(const PackageMap& other_map) {
  impl_->AddMap(*other_map.impl_);
}

void PackageMap::AddRemote(std::string package_name, RemoteParams params) {
  drake::log()->trace("PackageMap.AddRemote('{}', '{}', ...)", package_name,
                      params.urls.empty() ? "" : params.urls.front());

  // Validate our arguments.
  if (params.urls.empty()) {
    throw std::logic_error(
        fmt::format("PackageMap::AddRemote on '{}' requires at least one URL",
                    package_name));
  }
  for (const std::string_view url : params.urls) {
    if (!((url.substr(0, 8) == "https://") || (url.substr(0, 7) == "http://") ||
          (url.substr(0, 7) == "file://"))) {
      throw std::logic_error(
          fmt::format("PackageMap::AddRemote on '{}' has unsupported URL '{}'",
                      package_name, url));
    }
  }
  if (!((params.sha256.size() == 64) &&
        (std::all_of(params.sha256.begin(), params.sha256.end(), [](char ch) {
          return std::isxdigit(ch);
        })))) {
    throw std::logic_error(
        fmt::format("PackageMap::AddRemote on '{}' has invalid sha256 '{}'",
                    package_name, params.sha256));
  }
  if (params.archive_type.has_value()) {
    // The `known_types` is defined by what `package_downloader` can handle,
    // which is defined by `shutil.get_unpack_formats()`.
    const std::initializer_list<const char*> known_types = {
        "zip", "tar", "gztar", "bztar", "xztar"};
    if (std::count(known_types.begin(), known_types.end(),
                   *params.archive_type) == 0) {
      throw std::logic_error(fmt::format(
          "PackageMap::AddRemote on '{}' has unsupported archive type '{}'",
          package_name, *params.archive_type));
    }
  }

  // We've finished checking that the params are well-formed.
  auto package_data = PackageData::MakeRemote(std::move(params));

  // If the user has denied networking AND there are no file:// urls AND the
  // package is not yet cached, then we should fail-fast.
  if (GetAllowedUrls(package_data.remote_params().urls).empty()) {
    if (!package_data.FindInCache()) {
      throw std::runtime_error(fmt::format(
          "PackageMap::AddRemote on '{}' only provides network URLs (i.e., no "
          "file:// fallbacks) but PackageMap networking has been disabled via "
          "the DRAKE_ALLOW_NETWORK environment variable and the download cache "
          "does not contain any matching file",
          package_name));
    }
  }

  // Add it now. Emplace will handle rejection of duplicates.
  impl_->Emplace(package_name, std::move(package_data));
}

bool PackageMap::Contains(const std::string& package_name) const {
  return impl_->map().find(package_name) != impl_->map().end();
}

void PackageMap::Remove(const std::string& package_name) {
  impl_->Remove(package_name);
}

void PackageMap::SetDeprecated(const std::string& package_name,
                               std::optional<std::string> deprecated_message) {
  DRAKE_THROW_UNLESS(Contains(package_name));
  impl_->SetDeprecated(package_name, std::move(deprecated_message));
}

int PackageMap::size() const {
  return impl_->map().size();
}

std::optional<std::string> PackageMap::GetDeprecated(
    const std::string& package_name) const {
  DRAKE_THROW_UNLESS(Contains(package_name));
  return impl_->map().at(package_name).get_deprecation();
}

std::vector<std::string> PackageMap::GetPackageNames() const {
  std::vector<std::string> package_names;
  package_names.reserve(impl_->map().size());
  for (const auto& [package_name, _] : impl_->map()) {
    package_names.push_back(package_name);
  }
  return package_names;
}

const std::string& PackageMap::GetPath(
    const std::string& package_name,
    std::optional<std::string>* deprecated_message) const {
  DRAKE_THROW_UNLESS(Contains(package_name));
  const auto& data = impl_->map().at(package_name);

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

  // If this is a remote package and we haven't fetched it yet, do that now.
  return data.GetPathWithAutomaticFetching(package_name);
}

std::string PackageMap::ResolveUrl(const std::string& url) const {
  drake::internal::DiagnosticPolicy diagnostic_policy;
  const internal::ResolveUriResult resolved =
      internal::ResolveUri(diagnostic_policy, url, *this, {});
  return resolved.GetStringPathIfExists();
}

void PackageMap::PopulateFromFolder(const fs::path& path) {
  this->PopulateFromFolder(path.string());
}

void PackageMap::PopulateFromFolder(const std::string& path) {
  DRAKE_THROW_UNLESS(!path.empty());
  CrawlForPackages(path);
}

void PackageMap::PopulateFromFolder(const char* path) {
  DRAKE_THROW_UNLESS(path != nullptr);
  this->PopulateFromFolder(std::string(path));
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
      const PackageData& existing_data = impl_->map().at(package_name);
      auto local = PackageData::MakeLocal(package_path);
      std::string error;
      if (!existing_data.CanMerge(local, &error)) {
        drake::log()->warn(
            "PackageMap is ignoring newly-found path for package '{}' {}",
            package_name, error);
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

void PackageMap::AddPackageXml(const fs::path& filename) {
  this->AddPackageXml(filename.string());
}

void PackageMap::AddPackageXml(const std::string& filename) {
  DRAKE_THROW_UNLESS(!filename.empty());
  const auto [package_name, deprecated_message] =
      ParsePackageManifest(filename);
  const std::string package_path = GetParentDirectory(filename);
  Add(package_name, package_path);
  SetDeprecated(package_name, deprecated_message);
}

void PackageMap::AddPackageXml(const char* filename) {
  DRAKE_THROW_UNLESS(filename != nullptr);
  this->AddPackageXml(std::string(filename));
}

std::string PackageMap::to_string() const {
  std::string result{"PackageMap:\n"};
  if (size() == 0) {
    result.append("  [EMPTY!]\n");
  }
  for (const auto& [package_name, data] : impl_->map()) {
    result.append(
        fmt::format("  - {}: {}\n", package_name, data.display_path()));
  }
  return result;
}

std::ostream& operator<<(std::ostream& out, const PackageMap& package_map) {
  return out << fmt::to_string(package_map);
}

}  // namespace multibody
}  // namespace drake
