#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt_ostream.h"

namespace drake {
namespace multibody {

/** Maps ROS package names to their full path on the local file system. It is
used by the SDF and URDF parsers when parsing files that reference ROS packages
for resources like mesh files. */
class PackageMap final {
 public:
  /** A constructor that initializes a default map containing only the top-level
  `drake` manifest. See PackageMap::MakeEmpty() to create an empty map. */
  PackageMap();

  /** @name Implements CopyConstructible, CopyAssignable, MoveConstructible,
  MoveAssignable */
  //@{
  PackageMap(const PackageMap&);
  PackageMap& operator=(const PackageMap&);
  PackageMap(PackageMap&&);
  PackageMap& operator=(PackageMap&&);
  //@}

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

  ///@}

  /** @name Functions for adding packages to the map */
  ///@{

  /** Adds package `package_name` and its path, `package_path`. Throws if
  `package_name` is already present in this PackageMap with a different path, or
  if `package_path` does not exist. */
  void Add(const std::string& package_name, const std::string& package_path);

  /** Adds package->path mappings from another PackageMap `other_map`. Throws
  if the other PackageMap contains the same package with a different path. */
  void AddMap(const PackageMap& other_map);

  /** Adds an entry into this PackageMap for the given `package.xml` filename.
  Throws if `filename` does not exist or its embedded name already exists in
  this map. */
  void AddPackageXml(const std::string& filename);

  /** Crawls down the directory tree starting at `path` searching for
  directories containing the file `package.xml`. For each of these directories,
  this method adds a new entry into this PackageMap where the key is the package
  name as specified within `package.xml` and the directory's path is the value.
  If a package already known by the PackageMap is found again with a conflicting
  path, a warning is logged and the original path is kept. If the path does not
  exist or is unreadable, a warning is logged. */
  void PopulateFromFolder(const std::string& path);

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

  friend std::ostream& operator<<(std::ostream& out,
                                  const PackageMap& package_map);

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

  /* This method is the same as Add() except if package_name is already present
  with a different path, then this method prints a warning and returns false
  without adding the new path. Returns true otherwise. */
  bool AddPackageIfNew(const std::string& package_name,
                       const std::string& path);

  /* Our member data is forward declared to hide implementation details. */
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::multibody::PackageMap> : drake::ostream_formatter {};
}  // namespace fmt
