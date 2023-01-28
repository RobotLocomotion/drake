#pragma once

#include <initializer_list>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {

/// Maps ROS package names to their full path on the local file system. It is
/// used by the SDF and URDF parsers when parsing files that reference ROS
/// packages for resources like mesh files.
class PackageMap final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PackageMap)

  /// A constructor that initializes a default map containing only the top-
  /// level `drake` manifest.
  PackageMap();

  /// A factory method that initializes an empty map.
  static class PackageMap MakeEmpty();

  /// Adds package @p package_name and its path, @p package_path.
  /// Throws if @p package_name is already present in this PackageMap with a
  /// different path, or if @p package_path does not exist.
  void Add(const std::string& package_name, const std::string& package_path);

  /// Adds package->path mappings from another PackageMap @p other_map. Throws
  /// if the other PackageMap contains the same package with a different path.
  void AddMap(const PackageMap& other_map);

  /// Returns true if and only if this PackageMap contains @p package_name.
  bool Contains(const std::string& package_name) const;

  /// Removes package @p package_name and its previously added path.
  /// Throws if @p package_name is not present in this PackageMap.
  void Remove(const std::string& package_name);

  /// Sets or clears the deprecation message for package @p package_name. A
  /// @p deprecated_message value of std::nullopt implies no deprecation. Aborts
  /// if no package named @p package_name exists in this PackageMap.
  void SetDeprecated(const std::string& package_name,
      std::optional<std::string> deprecated_message);

  /// Returns the number of entries in this PackageMap.
  int size() const;

  /// Returns the deprecation message for package @p package_name if it has
  /// been set as deprecated. A value of std::nullopt implies no deprecation.
  /// Aborts if no package named @p package_name exists in this PackageMap.
  std::optional<std::string> GetDeprecated(
      const std::string& package_name) const;

  /// Returns the package names in this PackageMap. The order of package names
  /// returned is unspecified.
  std::vector<std::string> GetPackageNames() const;

  /// Obtains the path associated with package @p package_name. Aborts if no
  /// package named @p package_name exists in this PackageMap.
  ///
  /// @param[out] deprecated_message When passed as nullptr (it's default
  /// value), then in case the @p package_name is deprecated a deprecation
  /// message will be logged. When passed as non-nullptr the deprecation
  /// message will be output into this argument and will not be logged;
  /// if the @p package_name is not deprecated, the message will be set to
  /// nullopt.
  const std::string& GetPath(
      const std::string& package_name,
      std::optional<std::string>* deprecated_message = nullptr) const;

  /// Adds an entry into this PackageMap for the given `package.xml` filename.
  /// Throws if @p filename does not exist or its embedded name already exists
  /// in this map.
  void AddPackageXml(const std::string& filename);

  /// Crawls down the directory tree starting at @p path searching for
  /// directories containing the file `package.xml`. For each of these
  /// directories, this method adds a new entry into this PackageMap where the
  /// key is the package name as specified within `package.xml` and the
  /// directory's path is the value.
  /// If a package already known by the PackageMap is found again with a
  /// conflicting path, a warning is logged and the original path is kept.
  /// If the path does not exist or is unreadable, a warning is logged.
  void PopulateFromFolder(const std::string& path);

  /// Obtains one or more paths from environment variable
  /// @p environment_variable. Crawls downward through the directory tree(s)
  /// starting from the path(s) searching for `package.xml` files. For each of
  /// these files, this method adds a new entry into this PackageMap where the
  /// key is the package name as specified within `package.xml` and the value is
  /// the path to the `package.xml` file. Multiple paths can be specified by
  /// separating them using the ':' symbol. For example, the environment
  /// variable can contain [path 1]:[path 2]:[path 3] to search three different
  /// paths.
  ///
  /// If a package already known by the PackageMap is found again with a
  /// conflicting path, a warning is logged and the original path is kept.
  ///
  /// If a path does not exist or is unreadable, a warning is logged.
  ///
  /// @warning This function must not be used when populating manifests from
  /// the ROS_PACKAGE_PATH environment variable. It will throw an exception
  /// when that is attempted. Instead, use PopulateFromRosPackagePath().
  void PopulateFromEnvironment(const std::string& environment_variable);

  /// Obtains one or more paths from the ROS_PACKAGE_PATH environment variable.
  /// Semantics are similar to PopulateFromEnvironment(), except that ROS-style
  /// crawl termination semantics are enabled, which means that subdirectories
  /// of already-identified packages are not searched, and neither are
  /// directories which contain any of the following marker files:
  /// - AMENT_IGNORE
  /// - CATKIN_IGNORE
  /// - COLCON_IGNORE
  void PopulateFromRosPackagePath();

  friend std::ostream& operator<<(std::ostream& out,
                                  const PackageMap& package_map);

 private:
  // Information about a package.
  struct PackageData {
    // Directory in which the manifest resides.
    std::string path;
    // Optional message declaring deprecation of the package.
    std::optional<std::string> deprecated_message;
  };

  // A constructor that initializes a map by parsing a list of package.xml
  // file paths.
  PackageMap(std::initializer_list<std::string> manifest_paths);

  // Recursively crawls through @p path looking for package.xml files. Adds
  // the packages defined by these package.xml files to this PackageMap.
  //
  // @param[in] stop_at_package When passed true, do not crawl into
  // subdirectories of packages which have already been found.
  // @param[in] stop_markers When a directory contains one or more files or
  // directories with one of the given names, do not crawl into that directory
  // or any subdirectories when searching for packages.
  void CrawlForPackages(const std::string& path,
      bool stop_at_package = false,
      const std::vector<std::string_view>& stop_markers = {});

  // This method is the same as Add() except if package_name is already present
  // with a different path, then this method prints a warning and returns false
  // without adding the new path. Returns true otherwise.
  bool AddPackageIfNew(const std::string& package_name,
      const std::string& path);

  // The key is the name of a ROS package and the value is a struct containing
  // information about that package.
  std::map<std::string, struct PackageData> map_;
};

}  // namespace multibody
}  // namespace drake
