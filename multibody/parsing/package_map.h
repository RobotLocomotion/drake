#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {

/// Maps ROS package names to their full path on the local file system. It is
/// used by the SDF and URDF parsers when parsing files that reference ROS
/// packages for resources like mesh files.
class PackageMap {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PackageMap)

  /// A constructor that initializes an empty map.
  PackageMap();

  /// Adds package @p package_name and its path, @p package_path.
  /// Throws if @p package_name is already present in this PackageMap, or
  /// if @p package_path does not exist.
  void Add(const std::string& package_name, const std::string& package_path);

  /// Returns true if and only if this PackageMap contains @p package_name.
  bool Contains(const std::string& package_name) const;

  /// Returns the number of entries in this PackageMap.
  int size() const;

  /// Obtains the path associated with package @p package_name. Aborts if no
  /// package named @p package_name exists in this PackageMap.
  const std::string& GetPath(const std::string& package_name) const;

  /// Adds an entry into this PackageMap for the given `package.xml` filename.
  /// Throws if @p package_xml_filename does not exist or its embedded name
  /// already exists in this map.
  void AddPackageXml(const std::string& package_xml_filename);

  /// Crawls down the directory tree starting at @p path searching for
  /// directories containing the file `package.xml`. For each of these
  /// directories, this method adds a new entry into this PackageMap where the
  /// key is the package name as specified within `package.xml` and the
  /// directory's path is the value.
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
  void PopulateFromEnvironment(const std::string& environment_variable);

  /// Crawls up the directory tree from @p model_file to `drake`
  /// searching for `package.xml` files. Adds the packages described by these
  /// `package.xml` files. If @p model_file is not in `drake`, this
  /// method returns without doing anything.
  ///
  /// @param[in] model_file The model file whose directory is the start of the
  /// search for `package.xml` files. This file must be an SDF or URDF file.
  void PopulateUpstreamToDrake(const std::string& model_file);

  friend std::ostream& operator<<(std::ostream& out,
                                  const PackageMap& package_map);

 private:
  // Recursively crawls through @p path looking for package.xml files. Adds
  // the packages defined by these package.xml files to this PackageMap.
  // Multiple paths can be searched by separating them using the ':' symbol. In
  // other words, @p path can be [path 1]:[path 2]:[path 3] to crawl through
  // three different paths.
  void CrawlForPackages(const std::string& path);

  // This method is the same as Add() except it first checks to ensure that
  // package_name is not already in this PackageMap. If it is not, this
  // method prints a warning and returns.
  void AddPackageIfNew(const std::string& package_name,
      const std::string& path);

  // Recursively searches up the directory path searching for package.xml files.
  // The @p directory must be a child of @p stop_at_directory.  Stops searching
  // when the search directory is not longer than @p stop_at_directory.
  // Adds the packages defined by these package.xml files to this PackageMap.
  // This method is intended to be called by PopulateUpstreamToDrake().
  void PopulateUpstreamToDrakeHelper(
      const std::string& directory,
      const std::string& stop_at_directory);

  // The key is the name of a ROS package and the value is the package's
  // directory.
  std::map<std::string, std::string> map_;
};

}  // namespace multibody
}  // namespace drake
