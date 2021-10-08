#include "drake/multibody/parsing/package_map.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"

using std::map;
using std::string;

namespace drake {
namespace multibody {
namespace {

string GetTestDataRoot() {
  const string desired_dir =
      "drake/multibody/parsing/test/package_map_test_packages/";
  const string contained_file =
      "package_map_test_package_a/package.xml";
  const string absolute_file_path = FindResourceOrThrow(
      desired_dir + contained_file);
  return absolute_file_path.substr(
      0, absolute_file_path.size() - contained_file.size());
}

void VerifyMatch(const PackageMap& package_map,
    const map<string, string>& expected_packages) {
  EXPECT_EQ(package_map.size(), static_cast<int>(expected_packages.size()));
  for (const auto& [package_name, package_path] : expected_packages) {
    ASSERT_TRUE(package_map.Contains(package_name));
    EXPECT_EQ(package_map.GetPath(package_name), package_path);
  }

  std::map<std::string, int> package_name_counts;
  for (const auto& package_name : package_map.GetPackageNames()) {
    package_name_counts[package_name]++;
  }
  // Confirm that every package name occurs only once, and is in the expected
  // packages.
  for (const auto& [package_name, count] : package_name_counts) {
    ASSERT_EQ(count, 1);
    ASSERT_EQ(expected_packages.count(package_name), 1);
  }
  // Confirm that every expected package is in the set of package names.
  for (const auto& [package_name, path] : expected_packages) {
    unused(path);
    ASSERT_EQ(package_name_counts.count(package_name), 1);
  }
}

void VerifyMatchWithTestDataRoot(const PackageMap& package_map) {
  const string root_path = GetTestDataRoot();
  map<string, string> expected_packages = {
    {"package_map_test_package_a", root_path +
        "package_map_test_package_a/"},
    {"package_map_test_package_b", root_path +
        "package_map_test_package_b/"},
    {"package_map_test_package_c", root_path +
        "package_map_test_package_set/package_map_test_package_c/"},
    {"package_map_test_package_d", root_path +
        "package_map_test_package_set/package_map_test_package_d/"},
  };
  VerifyMatch(package_map, expected_packages);
}

// Tests that the PackageMap can be manually populated and unpopulated.
GTEST_TEST(PackageMapTest, TestManualPopulation) {
  filesystem::create_directory("package_foo");
  filesystem::create_directory("package_bar");
  filesystem::create_directory("package_baz");
  map<string, string> expected_packages = {
    {"package_foo", "package_foo"},
    {"my_package", "package_bar"}
  };

  // Add packages + paths.
  PackageMap package_map = PackageMap::MakeEmpty();
  for (const auto& [package, path] : expected_packages) {
    package_map.Add(package, path);
  }

  VerifyMatch(package_map, expected_packages);

  // Adding a duplicate package with the same path is OK.
  package_map.Add("package_foo", "package_foo");
  // Adding a duplicate package with a different path throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.Add("package_foo", "package_baz"), std::runtime_error,
      ".*conflicts with.*");
  // Adding a package with a nonexistent path throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.Add("garbage", "garbage"), std::runtime_error,
      ".*does not exist.*");

  VerifyMatch(package_map, expected_packages);

  // Remove packages + paths.
  map<string, string> expected_remaining_packages(expected_packages);
  for (const auto& it : expected_packages) {
    package_map.Remove(it.first);
    expected_remaining_packages.erase(it.first);
    VerifyMatch(package_map, expected_remaining_packages);
  }

  VerifyMatch(package_map, std::map<string, string>());

  EXPECT_THROW(package_map.Remove("package_baz"), std::runtime_error);
}

// Tests that PackageMaps can be combined via AddMap.
GTEST_TEST(PackageMapTest, TestAddMap) {
  filesystem::create_directory("package_foo");
  filesystem::create_directory("package_bar");
  filesystem::create_directory("package_baz");
  map<string, string> expected_packages_1 = {
    {"package_foo", "package_foo"},
    {"package_bar", "package_bar"}
  };
  map<string, string> expected_packages_2 = {
    {"package_foo", "package_foo"},
    {"package_baz", "package_baz"}
  };
  map<string, string> expected_packages_combined = {
    {"package_foo", "package_foo"},
    {"package_bar", "package_bar"},
    {"package_baz", "package_baz"}
  };
  map<string, string> expected_packages_conflicting = {
    {"package_foo", "package_foo"},
    {"package_baz", "package_bar"}
  };

  // Populate package maps.
  PackageMap package_map_1 = PackageMap::MakeEmpty();
  for (const auto& [package, path] : expected_packages_1) {
    package_map_1.Add(package, path);
  }

  VerifyMatch(package_map_1, expected_packages_1);

  PackageMap package_map_2 = PackageMap::MakeEmpty();
  for (const auto& [package, path] : expected_packages_2) {
    package_map_2.Add(package, path);
  }

  VerifyMatch(package_map_2, expected_packages_2);

  PackageMap package_map_conflicting = PackageMap::MakeEmpty();
  for (const auto& [package, path] : expected_packages_conflicting) {
    package_map_conflicting.Add(package, path);
  }

  VerifyMatch(package_map_conflicting, expected_packages_conflicting);

  // Combine package maps with a matching duplicate package + path.
  PackageMap package_map_1_copy = package_map_1;
  package_map_1_copy.AddMap(package_map_2);

  VerifyMatch(package_map_1_copy, expected_packages_combined);

  // Combining package maps with a conflicting package + path throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map_1_copy.AddMap(package_map_conflicting), std::runtime_error,
      ".*conflicts with.*");
}

// Tests that PackageMap can be populated by a package.xml.
GTEST_TEST(PackageMapTest, TestPopulateFromXml) {
  const string xml_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/package.xml");
  const string xml_dirname =
      filesystem::path(xml_filename).parent_path().string();
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.AddPackageXml(xml_filename);

  map<string, string> expected_packages = {
    {"package_map_test_package_a", xml_dirname},
  };
  VerifyMatch(package_map, expected_packages);

  // Adding the same package.xml again is OK, since it provides an identical
  // package name + path.
  package_map.AddPackageXml(xml_filename);
  VerifyMatch(package_map, expected_packages);

  // Adding a conflicting package.xml with the same package name but different
  // path throws.
  const string conflicting_xml_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/package_map_test_package_conflicting/"
      "package.xml");
  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.AddPackageXml(conflicting_xml_filename), std::runtime_error,
      ".*conflicts with.*");
}

// Tests that PackageMap can be populated by crawling down a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  const string root_path = GetTestDataRoot();
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.PopulateFromFolder(root_path);
  VerifyMatchWithTestDataRoot(package_map);
}

// Tests that PackageMap can handle being populated by crawling down a directory
// tree when it is provided a path with extraneous trailing slashes.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolderExtraTrailingSlashes) {
  const string root_path = GetTestDataRoot();
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.PopulateFromFolder(root_path + "///////");
  VerifyMatchWithTestDataRoot(package_map);
}

// Tests that PackageMap can be populated by crawling up a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateUpstreamToDrake) {
  const string root_path = GetTestDataRoot();
  const string sdf_file_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/"
      "sdf/test_model.sdf");

  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.PopulateUpstreamToDrake(sdf_file_name);

  map<string, string> expected_packages = {
    {"package_map_test_package_a", root_path + "package_map_test_package_a"}
  };

  VerifyMatch(package_map, expected_packages);

  // Call it again to exercise the "don't add things twice" code.
  package_map.PopulateUpstreamToDrake(sdf_file_name);
  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can be populated from an env var.
GTEST_TEST(PackageMapTest, TestPopulateFromEnvironment) {
  PackageMap package_map = PackageMap::MakeEmpty();

  // Test a null environment.
  package_map.PopulateFromEnvironment("FOOBAR");
  EXPECT_EQ(package_map.size(), 0);

  // Test an empty environment.
  ::setenv("FOOBAR", "", 1);
  package_map.PopulateFromEnvironment("FOOBAR");
  EXPECT_EQ(package_map.size(), 0);

  // Test three environment entries, concatenated:
  // - one bad path
  // - one good path
  // - one empty path
  const std::string value = "/does/not/exist:" + GetTestDataRoot() + ":";
  ::setenv("FOOBAR", value.c_str(), 1);
  package_map.PopulateFromEnvironment("FOOBAR");
  VerifyMatchWithTestDataRoot(package_map);
}

// Tests that PackageMap's streaming to-string operator works.
GTEST_TEST(PackageMapTest, TestStreamingToString) {
  filesystem::create_directory("package_foo");
  filesystem::create_directory("package_bar");
  map<string, string> expected_packages = {
    {"package_foo", "package_foo"},
    {"my_package", "package_bar"}
  };

  PackageMap package_map = PackageMap::MakeEmpty();
  for (const auto& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }

  std::stringstream string_buffer;
  string_buffer << package_map;
  const std::string resulting_string = string_buffer.str();

  // The following simply tests that the package names and their relative paths
  // exist in the resulting string. It does not check the literal path since
  // that's system dependent or the actual formatting of the text.
  for (const auto& it : expected_packages) {
    EXPECT_NE(resulting_string.find(it.first), std::string::npos);
    EXPECT_NE(resulting_string.find(it.second), std::string::npos);
  }

  // Verifies that there are three lines in the resulting string.
  EXPECT_EQ(std::count(resulting_string.begin(), resulting_string.end(), '\n'),
            3);
}

// Tests that PackageMap is parsing deprecation messages
GTEST_TEST(PackageMapTest, TestDeprecation) {
  const
  std::map<std::string, std::optional<std::string>> expected_deprecations = {
    {
      "package_map_test_package_b",
      "package_map_test_package_b is deprecated, and will be removed on or "
          "around 2038-01-19. Please use the 'drake' package instead."
    },
    {"package_map_test_package_d", ""},
  };
  const string root_path = GetTestDataRoot();
  PackageMap package_map;
  package_map.PopulateFromFolder(root_path);
  for (const auto& package_name : package_map.GetPackageNames()) {
    const auto expected_message = expected_deprecations.find(package_name);
    if (expected_message != expected_deprecations.end()) {
      EXPECT_EQ(package_map.GetDeprecated(package_name),
                expected_message->second);
    } else {
      EXPECT_FALSE(package_map.GetDeprecated(package_name).has_value());
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
