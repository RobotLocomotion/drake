#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"
#include "drake/common/name_value.h"

namespace drake {
namespace multibody {

/** Maps ROS package names to their full path on the local file system. It is
used by the SDF and URDF parsers when parsing files that reference ROS packages
for resources like mesh files. This class can also download remote packages from
the internet on an as-needed basis via AddRemote(). */
class PackageMap final {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(PackageMap);

  /** A constructor that initializes a default map containing only the top-level
  `drake` manifest. See PackageMap::MakeEmpty() to create an empty map. */
  PackageMap();

  ~PackageMap();

  /** A factory method that initializes an empty map. */
  static class PackageMap MakeEmpty();

  /** @name Functions for looking up packages in the map */
  ///@{

  /** Returns the number of entries in this PackageMap. */
  int size() const;

  /** Returns the package names in this PackageMap. The order of package names
  returned is unspecified. */
  std::vector<std::string> GetPackageNames() const;

  /** Returns true if and only if this PackageMap contains `package_name`. */
  bool Contains(const std::string& package_name) const;

  /** Obtains the path associated with package `package_name`. Aborts if no
  package named `package_name` exists in this PackageMap.
  @param[out] deprecated_message When passed as nullptr (its default value),
  then in case the `package_name` is deprecated a deprecation message will be
  logged. When passed as non-nullptr the deprecation message will be output into
  this argument and will not be logged; if the `package_name` is not deprecated,
  the message will be set to nullopt. */
  const std::string& GetPath(
      const std::string& package_name,
      std::optional<std::string>* deprecated_message = nullptr) const;

  /** Returns a resolved path for `url`. URL schemes are either `file://` for
  local files or `package://` (or `model://`).

  @throws std::exception if the url cannot be resolved. */
  std::string ResolveUrl(const std::string& url) const;

  ///@}

  /** @name Functions for adding packages to the map */
  ///@{

  /** Adds package `package_name` and its path, `package_path`. Throws if
  `package_name` is already present in this PackageMap with a different path, or
  if `package_path` does not exist. */
  void Add(const std::string& package_name,
           const std::filesystem::path& package_path);

  /** Legacy overload of Add() that accepts a string instead of a
  std::filesystem::path.
  @exclude_from_pydrake_mkdoc{Only the fs::path overload is bound.} */
  void Add(const std::string& package_name, const std::string& package_path);

  /** Legacy overload of Add() that accepts a null-terminated C string instead
  of a std::filesystem::path.
  @exclude_from_pydrake_mkdoc{Only the fs::path overload is bound.} */
  void Add(const std::string& package_name, const char* package_path);

  /** Adds all packages from `other_map` into `this`. Throws if `other` contains
  a package with the same `package_name` as one already in this map but with
  incompatible details (e.g., a different local path). */
  void AddMap(const PackageMap& other_map);

  /** Adds an entry into this PackageMap for the given `package.xml` filename.
  Throws if `filename` does not exist or its embedded name already exists in
  this map. */
  void AddPackageXml(const std::filesystem::path& filename);

  /** Legacy overload of AddPackageXml() that accepts a string instead of a
  std::filesystem::path.
  @exclude_from_pydrake_mkdoc{Only the fs::path overload is bound.} */
  void AddPackageXml(const std::string& filename);

  /** Legacy overload of AddPackageXml() that accepts a null-terminated C string
  instead of a std::filesystem::path.
  @exclude_from_pydrake_mkdoc{Only the fs::path overload is bound.} */
  void AddPackageXml(const char* filename);

  /** Parameters used for AddRemote(). */
  struct RemoteParams {
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(urls));
      a->Visit(DRAKE_NVP(sha256));
      a->Visit(DRAKE_NVP(archive_type));
      a->Visit(DRAKE_NVP(strip_prefix));
    }

    /** Returns the JSON serialization of these params. */
    std::string ToJson() const;

    /** Equality operator. */
    friend bool operator==(const RemoteParams&, const RemoteParams&);

    /** The list of remote URLs for this resource. The urls are used in the
    other they appear here, so preferred mirror(s) should come first. Valid
    methods are "http://" or "https://" or "file://". */
    std::vector<std::string> urls;

    /** The cryptographic checksum of the file to be downloaded, as a
    64-character hexadecimal string. */
    std::string sha256;

    /** (Optional) The archive type of the downloaded file. Valid options are
    "zip", "tar", "gztar", "bztar", or "xztar". By default, the archive type is
    determined from the file extension of the URL; in case the URL has no
    filename extension, you should explicitly specify one here. */
    std::optional<std::string> archive_type;

    /** (Optional) A directory prefix to remove from the extracted files. In
    many cases, an archive will prefix all filenames with something like
    "package-v1.2.3/" so that it extracts into a convenient directory. This
    option will discard that common prefix when extracting the archive for the
    PackageMap. It is an error if the archive does not contain any diectory with
    this prefix, but if there are files outside of this directory they will be
    silently discarded. */
    std::optional<std::string> strip_prefix;
  };

  /** Adds an entry into this PackageMap for the given `package_name`, which
  will be downloaded from the internet (with local caching). The data will not
  be downloaded until necessary, i.e., when GetPath() is first called for the
  `package_name`. Throws if the `package_name` or `params` are invalid.
  Downloading requires a valid `/usr/bin/python3` interpreter, which will be
  invoked as a subprocess.

  See \ref allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
  option to disable network fetching. %AddRemote may still be used even with
  network fetching disabled -- in that case, the urls must contain a "file://"
  URL or the download cache must already contain a previously-downloaded copy
  of the package (with the same sha256 checksum). */
  void AddRemote(std::string package_name, RemoteParams params);

  /** Crawls down the directory tree starting at `path` searching for
  directories containing the file `package.xml`. For each of these directories,
  this method adds a new entry into this PackageMap where the key is the package
  name as specified within `package.xml` and the directory's path is the value.
  If a package already known by the PackageMap is found again with a conflicting
  path, a warning is logged and the original path is kept. If the path does not
  exist or is unreadable, a warning is logged. */
  void PopulateFromFolder(const std::filesystem::path& path);

  /** Legacy overload of PopulateFromFolder() that accepts a string instead of a
  std::filesystem::path.
  @exclude_from_pydrake_mkdoc{Only the fs::path overload is bound.} */
  void PopulateFromFolder(const std::string& path);

  /** Legacy overload of PopulateFromFolder() that accepts a null-terminated C
  string instead of a std::filesystem::path.
  @exclude_from_pydrake_mkdoc{Only the fs::path overload is bound.} */
  void PopulateFromFolder(const char* path);

  /** Obtains one or more paths from environment variable
  `environment_variable`. Crawls downward through the directory tree(s) starting
  from the path(s) searching for `package.xml` files. For each of these files,
  this method adds a new entry into this PackageMap where the key is the package
  name as specified within `package.xml` and the value is the path to the
  `package.xml` file. Multiple paths can be specified by separating them using
  the ':' symbol. For example, the environment variable can contain [path
  1]:[path 2]:[path 3] to search three different paths.

  If a package already known by the PackageMap is found again with a conflicting
  path, a warning is logged and the original path is kept.

  If a path does not exist or is unreadable, a warning is logged.

  @warning This function must not be used when populating manifests from the
  ROS_PACKAGE_PATH environment variable. It will throw an exception when that is
  attempted. Instead, use PopulateFromRosPackagePath(). */
  void PopulateFromEnvironment(const std::string& environment_variable);

  /** Obtains one or more paths from the ROS_PACKAGE_PATH environment variable.
  Semantics are similar to PopulateFromEnvironment(), except that ROS-style
  crawl termination semantics are enabled, which means that subdirectories of
  already-identified packages are not searched, and neither are directories
  which contain any of the following marker files:
  - AMENT_IGNORE
  - CATKIN_IGNORE
  - COLCON_IGNORE */
  void PopulateFromRosPackagePath();
  ///@}

  /** @name Functions for modifying packages already in the map */
  ///@{

  /** Returns the deprecation message for package `package_name` if it has been
  set as deprecated. A value of std::nullopt implies no deprecation. Aborts if
  no package named `package_name` exists in this PackageMap. */
  std::optional<std::string> GetDeprecated(
      const std::string& package_name) const;

  /** Sets or clears the deprecation message for package `package_name`. A
  `deprecated_message` value of std::nullopt implies no deprecation. Aborts if
  no package named `package_name` exists in this PackageMap. */
  void SetDeprecated(const std::string& package_name,
                     std::optional<std::string> deprecated_message);

  /** Removes package `package_name` and its previously added path. Throws if
  `package_name` is not present in this PackageMap. */
  void Remove(const std::string& package_name);

  ///@}

  std::string to_string() const;

 private:
  /* A constructor that creates an empty map . */
  explicit PackageMap(std::nullopt_t);

  /* Recursively crawls through `path` looking for package.xml files. Adds the
  packages defined by these package.xml files to this PackageMap.
  @param[in] stop_at_package When passed true, do not crawl into subdirectories
  of packages which have already been found.
  @param[in] stop_markers When a directory contains one or more files or
  directories with one of the given names, do not crawl into that directory or
  any subdirectories when searching for packages. */
  void CrawlForPackages(const std::string& path, bool stop_at_package = false,
                        const std::vector<std::string_view>& stop_markers = {});

  /* Our member data is forward declared to hide implementation details. */
  class Impl;
  std::unique_ptr<Impl> impl_;
};

DRAKE_DEPRECATED(
    "2026-06-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream& out, const PackageMap& package_map);

namespace internal {

/* (Internal use only) Parses the metadata from `tools/workspace/drake_models`
into the RemoteParams structure needed by PackageMap. */
PackageMap::RemoteParams GetDrakeModelsRemoteParams();

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::multibody, PackageMap, x, x.to_string())
