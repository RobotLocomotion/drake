#include "drake/multibody/parsing/package_map.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"

using std::map;
using std::string;

namespace drake {
namespace multibody {
namespace {

string GetTestDataRoot() {
  const string desired_dir =
      "drake/multibody/parsing/test/";
  const string contained_file =
      "package_map_test_packages/package_map_test_package_a/package.xml";
  const string absolute_file_path = FindResourceOrThrow(
      desired_dir + contained_file);
  return absolute_file_path.substr(
      0, absolute_file_path.size() - contained_file.size());
}

void VerifyMatch(const PackageMap& package_map,
    const map<string, string>& expected_packages) {
  EXPECT_EQ(package_map.size(), static_cast<int>(expected_packages.size()));
  for (const auto& path_entry : expected_packages) {
    const std::string& package_name = path_entry.first;
    const std::string& package_path = path_entry.second;

    ASSERT_TRUE(package_map.Contains(package_name));
    EXPECT_EQ(package_map.GetPath(package_name), package_path);
  }
}

void VerifyMatchWithTestDataRoot(const PackageMap& package_map) {
  const string root_path = GetTestDataRoot();
  map<string, string> expected_packages = {
    {"package_map_test_package_a", root_path +
        "package_map_test_packages/package_map_test_package_a/"},
    {"package_map_test_package_b", root_path +
        "package_map_test_packages/package_map_test_package_b/"},
    {"package_map_test_package_c", root_path +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_c/"},
    {"package_map_test_package_d", root_path +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_d/"},
    {"box_model", root_path +
        "box_package/"},
    // TODO(#10531) This should use `package://drake`.
    {"process_model_directives_test", root_path +
        "process_model_directives_test/"},
  };
  VerifyMatch(package_map, expected_packages);
}

// Tests that the PackageMap can be manually populated and unpopulated.
GTEST_TEST(PackageMapTest, TestManualPopulation) {
  filesystem::create_directory("package_foo");
  filesystem::create_directory("package_bar");
  map<string, string> expected_packages = {
    {"package_foo", "package_foo"},
    {"my_package", "package_bar"}
  };

  PackageMap package_map;
  for (const auto& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }

  VerifyMatch(package_map, expected_packages);

  map<string, string> expected_remaining_packages(expected_packages);
  for (const auto& it : expected_packages) {
    package_map.Remove(it.first);
    expected_remaining_packages.erase(it.first);
    VerifyMatch(package_map, expected_remaining_packages);
  }

  VerifyMatch(package_map, std::map<string, string>());

  EXPECT_THROW(package_map.Remove("package_baz"), std::runtime_error);
}

// Tests that PackageMap can be populated by a package.xml.
GTEST_TEST(PackageMapTest, TestPopulateFromXml) {
  const string xml_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/package.xml");
  const string xml_dirname =
      filesystem::path(xml_filename).parent_path().string();
  PackageMap package_map;
  package_map.AddPackageXml(xml_filename);

  map<string, string> expected_packages = {
    {"package_map_test_package_a", xml_dirname},
  };
  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can be populated by crawling down a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  const string root_path = GetTestDataRoot();
  PackageMap package_map;
  package_map.PopulateFromFolder(root_path);
  VerifyMatchWithTestDataRoot(package_map);
}

// Tests that PackageMap can handle being populated by crawling down a directory
// tree when it is provided a path with extraneous trailing slashes.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolderExtraTrailingSlashes) {
  const string root_path = GetTestDataRoot();
  PackageMap package_map;
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

  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_name);

  map<string, string> expected_packages = {
    {"package_map_test_package_a",
        root_path + "package_map_test_packages/package_map_test_package_a"}
  };

  VerifyMatch(package_map, expected_packages);

  // Call it again to exercise the "don't add things twice" code.
  package_map.PopulateUpstreamToDrake(sdf_file_name);
  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can be populated from an env var.
GTEST_TEST(PackageMapTest, TestPopulateFromEnvironment) {
  PackageMap package_map;

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

  PackageMap package_map;
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

}  // namespace
}  // namespace multibody
}  // namespace drake
