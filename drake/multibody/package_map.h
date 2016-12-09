#pragma once

#include <map>
#include <string>

namespace drake {
namespace parsers {

class PackageMap {
 public:
  /// A constructor that initializes an empty map.
  PackageMap();

  /// Adds package @p package_name and its path, @p package_path. Aborts if
  /// @p package_name is already present in this PackageMap.
  void Add(const std::string& package_name, const std::string& package_path);

  /// Returns true if and only if this PackageMap contains @p package_map.
  bool Contains(const std::string& package_map);

  /// Obtains the path associated with package @p package_name. Aborts if no
  /// package named @p package_name exists in this PackageMap.
  std::string GetPath(const std::string& package_name);

  /// Crawls through @p path searching for directories containing the file
  /// `package.xml`. For each of these directories, this method adds a new entry
  /// into @p package_map where the key is the package name as specified within
  /// `package.xml` and the directory's path is the value.
  void PopulateFromFolder(const std::string& path);

  /// Obtains a path from environment variable @p environment_variable. Crawls
  /// through this path searching for directories containing the file
  /// `package.xml`. For each of these directories, this method adds a new entry
  /// into @p package_map where the key is the package name as specified within
  /// `package.xml` and the directory's path is the value.
  void PopulateFromEnvironment(const std::string& environment_variable);

 private:
  // Recursively crawls through @p path looking for package.xml files. Adds
  // the packages defined by these package.xml files to this PackageMap.
  void PackageMap::CrawlForPackages(const string& path);

  // The key is the name of a ROS package and the value is the package's
  // directory.
  std::map<std::string, std::string> map_;
}

}  // namespace parsers
}  // namespace drake
