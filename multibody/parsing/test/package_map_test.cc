#include "drake/multibody/parsing/package_map.h"

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/scope_exit.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"

using std::map;
using std::string;

namespace drake {
namespace multibody {
namespace {

namespace fs = std::filesystem;

// N.B. See also package_map_remote_test.cc for additional test cases.

string GetTestDataRoot() {
  const string desired_dir =
      "drake/multibody/parsing/test/package_map_test_packages/";
  const string contained_file = "package_map_test_package_a/package.xml";
  const string absolute_file_path =
      FindResourceOrThrow(desired_dir + contained_file);
  return absolute_file_path.substr(
      0, absolute_file_path.size() - contained_file.size());
}

void VerifyMatch(const PackageMap& package_map,
                 const map<string, string>& expected_packages) {
  EXPECT_EQ(package_map.size(), static_cast<int>(expected_packages.size()));
  for (const auto& [package_name, package_path] : expected_packages) {
    ASSERT_TRUE(package_map.Contains(package_name));
    std::optional<string> deprecation;
    EXPECT_EQ(package_map.GetPath(package_name, &deprecation), package_path);

    const bool should_be_deprecated =
        (package_name == "package_map_test_package_b") ||
        (package_name == "package_map_test_package_d");
    EXPECT_EQ(deprecation.has_value(), should_be_deprecated)
        << "for " << package_name;
    EXPECT_EQ(!deprecation.value_or("").empty(), should_be_deprecated)
        << "for " << package_name;
  }

  std::map<std::string, int> package_name_counts;
  for (const auto& package_name : package_map.GetPackageNames()) {
    package_name_counts[package_name]++;
  }
  // Confirm that every package name occurs only once, and is in the expected
  // packages.
  for (const auto& [package_name, count] : package_name_counts) {
    ASSERT_EQ(count, 1);
    ASSERT_TRUE(expected_packages.contains(package_name));
  }
  // Confirm that every expected package is in the set of package names.
  for (const auto& [package_name, path] : expected_packages) {
    unused(path);
    ASSERT_TRUE(package_name_counts.contains(package_name));
  }
}

void VerifyMatchWithTestDataRoot(const PackageMap& package_map) {
  const string root_path = GetTestDataRoot();
  map<string, string> expected_packages = {
      {"package_map_test_package_a",  // BR
       root_path + "package_map_test_package_a/"},
      {"package_map_test_package_aa",
       root_path + "package_map_test_package_a/package_map_test_package_aa/"},
      {"package_map_test_package_b",  // BR
       root_path + "package_map_test_package_b/"},
      {"package_map_test_package_c",
       root_path + "package_map_test_package_set/package_map_test_package_c/"},
      {"package_map_test_package_d",
       root_path + "package_map_test_package_set/package_map_test_package_d/"},
      {"package_map_test_package_e",  // BR
       root_path + "package_map_test_package_e/"},
  };
  VerifyMatch(package_map, expected_packages);
}

// We need to indirect self-move-assign through this function; doing it directly
// in the test code generates a compiler warning.
template <typename T>
void MoveAssign(T* target, T* donor) {
  *target = std::move(*donor);
}

// Tests the lifecycle operations.
GTEST_TEST(PackageMapTest, Lifecycle) {
  const PackageMap original;
  const int default_size = original.size();

  PackageMap copied(original);
  EXPECT_EQ(copied.size(), default_size);
  EXPECT_EQ(original.size(), default_size);

  PackageMap donor;
  EXPECT_EQ(donor.size(), default_size);
  PackageMap moved(std::move(donor));
  EXPECT_EQ(donor.size(), 0);
  EXPECT_EQ(moved.size(), default_size);

  auto copy_assigned = PackageMap::MakeEmpty();
  EXPECT_EQ(copy_assigned.size(), 0);
  copy_assigned = original;
  EXPECT_EQ(copy_assigned.size(), default_size);
  EXPECT_EQ(original.size(), default_size);

  auto move_assigned = PackageMap::MakeEmpty();
  EXPECT_EQ(move_assigned.size(), 0);
  move_assigned = std::move(moved);
  EXPECT_EQ(move_assigned.size(), default_size);
  EXPECT_EQ(moved.size(), 0);

  MoveAssign(&move_assigned, &move_assigned);
  EXPECT_EQ(move_assigned.size(), default_size);
}

// Tests that the PackageMap can be manually populated and unpopulated.
GTEST_TEST(PackageMapTest, TestManualPopulation) {
  fs::create_directory("package_foo");
  fs::create_directory("package_bar");
  fs::create_directory("package_baz");
  map<string, string> expected_packages = {{"package_foo", "package_foo"},
                                           {"my_package", "package_bar"},
                                           {"third", "package_baz"}};

  // Add packages + paths. Check all overloads.
  PackageMap package_map = PackageMap::MakeEmpty();
  int i = 0;
  for (const auto& [package, path] : expected_packages) {
    if (i % 3 == 0) {
      package_map.Add(package, fs::path{path});
    } else if (i % 3 == 1) {
      package_map.Add(package, std::string{path});
    } else {
      package_map.Add(package, path.c_str());
    }
    ++i;
  }
  EXPECT_EQ(i, 3);

  VerifyMatch(package_map, expected_packages);

  // Adding a duplicate package with the same path is OK.
  package_map.Add("package_foo", "package_foo");
  // Adding a duplicate package with a different path throws.
  DRAKE_EXPECT_THROWS_MESSAGE(package_map.Add("package_foo", "package_baz"),
                              ".*paths are not eq.*");
  // Adding a package with a nonexistent path throws.
  DRAKE_EXPECT_THROWS_MESSAGE(package_map.Add("garbage", "garbage"),
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

  EXPECT_THROW(package_map.Remove("never_existed"), std::runtime_error);
}

// Default-constructed maps must always be merge-able.
GTEST_TEST(PackageMapTest, AddDefaultConstructedMaps) {
  const PackageMap foo;
  const PackageMap bar;
  PackageMap dut;
  dut.AddMap(foo);
  dut.AddMap(bar);
  EXPECT_EQ(dut.size(), foo.size());
}

// Tests that PackageMaps can be combined via AddMap.
GTEST_TEST(PackageMapTest, TestAddMap) {
  fs::create_directory("package_foo");
  fs::create_directory("package_bar");
  fs::create_directory("package_baz");
  map<string, string> expected_packages_1 = {{"package_foo", "package_foo"},
                                             {"package_bar", "package_bar"}};
  map<string, string> expected_packages_2 = {{"package_foo", "package_foo"},
                                             {"package_baz", "package_baz"}};
  map<string, string> expected_packages_combined = {
      {"package_foo", "package_foo"},
      {"package_bar", "package_bar"},
      {"package_baz", "package_baz"}};
  map<string, string> expected_packages_conflicting = {
      {"package_foo", "package_foo"}, {"package_baz", "package_bar"}};

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
      package_map_1_copy.AddMap(package_map_conflicting),
      ".*paths are not eq.*");
}

// Tests that combining via AddMap retains deprecation information
GTEST_TEST(PackageMapTest, TestAddMapDeprecated) {
  PackageMap dut = PackageMap::MakeEmpty();
  dut.Add("default", ".");

  // The deprecation comes along while merging.
  PackageMap hats = PackageMap::MakeEmpty();
  hats.Add("hats", ".");
  hats.SetDeprecated("hats", "I like hats.");
  dut.AddMap(hats);
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.GetDeprecated("default").value_or(""), "");
  EXPECT_EQ(dut.GetDeprecated("hats").value_or(""), "I like hats.");

  // An *existing* deprecation on the dut remains intact when merging in
  // another map that doesn't have any deprecation.
  PackageMap temp = PackageMap::MakeEmpty();
  temp.Add("hats", ".");
  dut.AddMap(temp);
  EXPECT_EQ(dut.GetDeprecated("hats").value_or(""), "I like hats.");

  // Likewise even if the merged map is deprecated with some new message.
  temp.SetDeprecated("hats", "Ignored!");
  dut.AddMap(temp);
  EXPECT_EQ(dut.GetDeprecated("hats").value_or(""), "I like hats.");

  // However, merging a deprecated package onto an undeprecated does inherit
  // the deprecation.
  temp = PackageMap::MakeEmpty();
  temp.Add("default", ".");
  temp.SetDeprecated("default", "Oh nelly!");
  dut.AddMap(temp);
  EXPECT_EQ(dut.GetDeprecated("default").value_or(""), "Oh nelly!");
}

// Tests that PackageMap can be populated by a package.xml.
GTEST_TEST(PackageMapTest, TestPopulateFromXml) {
  const string xml_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/package.xml");
  const string xml_dirname = fs::path(xml_filename).parent_path().string();
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.AddPackageXml(fs::path{xml_filename});

  map<string, string> expected_packages = {
      {"package_map_test_package_a", xml_dirname},
  };
  VerifyMatch(package_map, expected_packages);

  // Adding the same package.xml again is OK, since it provides an identical
  // package name + path. Use a different overload for coverage.
  package_map.AddPackageXml(std::string{xml_filename});
  VerifyMatch(package_map, expected_packages);

  // Adding a conflicting package.xml with the same package name but different
  // path throws. Use a different overload for coverage.
  const string conflicting_xml_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/package_map_test_package_conflicting/"
      "package.xml");
  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.AddPackageXml(conflicting_xml_filename.c_str()),
      ".*paths are not eq.*");

  // Adding the same filesystem-canonical package.xml twice is not an error.
  fs::create_directory("alternative_package_a");
  fs::create_symlink(xml_filename, "alternative_package_a/package.xml");
  package_map.Add("package_map_test_package_a", "alternative_package_a");
}

// Tests that PackageMap can be populated by crawling down a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  const string root_path = GetTestDataRoot();
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.PopulateFromFolder(fs::path{root_path});
  VerifyMatchWithTestDataRoot(package_map);
}

// Tests that PackageMap can handle being populated by crawling down a directory
// tree when it is provided a path with extraneous trailing slashes.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolderExtraTrailingSlashes) {
  const string root_path = GetTestDataRoot();
  const string with_extra_slashes = root_path + "///////";

  // First with std::string.
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.PopulateFromFolder(with_extra_slashes);
  VerifyMatchWithTestDataRoot(package_map);

  // Again with c_str for coverage.
  package_map = PackageMap::MakeEmpty();
  package_map.PopulateFromFolder(with_extra_slashes.c_str());
  VerifyMatchWithTestDataRoot(package_map);
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

// Tests that PackageMap can be populated from the
// ROS_PACKAGE_PATH env var.
GTEST_TEST(PackageMapTest, TestPopulateFromRosPackagePath) {
  PackageMap package_map = PackageMap::MakeEmpty();

  // Test a null environment.
  package_map.PopulateFromRosPackagePath();
  EXPECT_EQ(package_map.size(), 0);

  // Test an empty environment.
  ::setenv("ROS_PACKAGE_PATH", "", 1);
  ScopeExit guard([]() {
    ::unsetenv("ROS_PACKAGE_PATH");
  });

  package_map.PopulateFromRosPackagePath();
  EXPECT_EQ(package_map.size(), 0);

  // Test three environment entries, concatenated:
  // - one bad path
  // - one good path
  // - one empty path
  const std::string root_path = GetTestDataRoot();
  const std::string value = "/does/not/exist:" + root_path + ":";
  ::setenv("ROS_PACKAGE_PATH", value.c_str(), 1);
  package_map.PopulateFromRosPackagePath();
  map<string, string> expected_packages = {
      {"package_map_test_package_a",  // BR
       root_path + "package_map_test_package_a/"},
      {"package_map_test_package_b",  // BR
       root_path + "package_map_test_package_b/"},
      {"package_map_test_package_c",
       root_path + "package_map_test_package_set/package_map_test_package_c/"},
      {"package_map_test_package_d",
       root_path + "package_map_test_package_set/package_map_test_package_d/"},
  };
  VerifyMatch(package_map, expected_packages);

  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.PopulateFromEnvironment("ROS_PACKAGE_PATH"),
      ".*use PopulateFromRosPackagePath.*");
}

// TODO(2026-06-01): delete TestStreamingToString
//  Tests that PackageMap's streaming to-string operator works.
GTEST_TEST(PackageMapTest, TestStreamingToString) {
  fs::create_directory("package_foo");
  fs::create_directory("package_bar");
  map<string, string> expected_packages = {{"package_foo", "package_foo"},
                                           {"my_package", "package_bar"}};

  PackageMap package_map = PackageMap::MakeEmpty();
  for (const auto& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }
  const std::string url = "file:///tmp/missing.zip";
  package_map.AddRemote("remote",
                        {.urls = {url}, .sha256 = std::string(64u, '0')});

  std::stringstream string_buffer;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  string_buffer << package_map;
#pragma GCC diagnostic pop
  const std::string resulting_string = string_buffer.str();

  // The following simply tests that the package names and their relative paths
  // exist in the resulting string. It does not check the literal path since
  // that's system dependent or the actual formatting of the text.
  for (const auto& it : expected_packages) {
    EXPECT_NE(resulting_string.find(it.first), std::string::npos);
    EXPECT_NE(resulting_string.find(it.second), std::string::npos);
  }
  EXPECT_NE(resulting_string.find(url), std::string::npos);

  // Verifies the number of lines in the resulting string.
  EXPECT_EQ(std::count(resulting_string.begin(), resulting_string.end(), '\n'),
            4);
}

// Tests that PackageMap can be converted to a string using its fmt formatter.
GTEST_TEST(PackageMapTest, ToStringFmtFormatter) {
  // Test the empty PackageMap case.
  EXPECT_EQ(fmt::to_string(PackageMap().MakeEmpty()),
            "PackageMap:\n  [EMPTY!]\n");

  fs::create_directory("package_foo");
  fs::create_directory("package_bar");
  map<string, string> expected_packages = {{"package_foo", "package_foo"},
                                           {"my_package", "package_bar"}};

  PackageMap package_map = PackageMap::MakeEmpty();
  for (const auto& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }
  const std::string url = "file:///tmp/missing.zip";
  package_map.AddRemote("remote",
                        {.urls = {url}, .sha256 = std::string(64u, '0')});

  const std::string resulting_string{fmt::to_string(package_map)};

  // The following simply tests that the package names and their relative paths
  // exist in the resulting string. It does not check the literal path since
  // that's system dependent or the actual formatting of the text.
  for (const auto& it : expected_packages) {
    EXPECT_NE(resulting_string.find(it.first), std::string::npos);
    EXPECT_NE(resulting_string.find(it.second), std::string::npos);
  }
  EXPECT_NE(resulting_string.find(url), std::string::npos);

  // Verifies the number of lines in the resulting string.
  EXPECT_EQ(std::count(resulting_string.begin(), resulting_string.end(), '\n'),
            4);
}

// ResolveUrl is just a thin wrapper around internal::ResolveUri (which is
// tested elsewhere). This test is just to ensure that the wrapper is working.
GTEST_TEST(PackageMapTest, TestResolveUrl) {
  const string xml_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/package.xml");
  PackageMap package_map = PackageMap::MakeEmpty();
  package_map.AddPackageXml(xml_filename);

  const string filename = package_map.ResolveUrl(
      "package://package_map_test_package_a/sdf/test_model.sdf");

  const string expected_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/package_map_test_packages/"
      "package_map_test_package_a/sdf/test_model.sdf");

  EXPECT_EQ(filename, expected_filename);

  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.ResolveUrl("package://bad_package_name/sdf/test_model.sdf"),
      ".*unknown package.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      package_map.ResolveUrl(
          "package://package_map_test_package_a/bad_filename.sdf"),
      ".*does not exist.*");
}

// Tests that PackageMap is parsing deprecation messages
GTEST_TEST(PackageMapTest, TestDeprecation) {
  const std::map<std::string, std::optional<std::string>> expected_deprecations{
      {"package_map_test_package_b",
       "package_map_test_package_b is deprecated, and will be removed on or "
       "around 2038-01-19. Please use the 'drake' package instead."},
      {"package_map_test_package_d", "(no explanation given)"},
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
