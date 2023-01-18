#include "drake/multibody/parsing/detail_path_utils.h"

#include <filesystem>
#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/unused.h"

using std::string;

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;

// This wraps the "ResolveUri()" device under test to throw for bad URIs.
std::string ResolveGoodUri(
    const std::string& uri,
    const PackageMap& package_map,
    const std::string& root_dir) {
  DiagnosticPolicy diagnostic;
  // Don't let warnings leak into spdlog; tests should always specifically
  // handle any warnings that appear.
  diagnostic.SetActionForWarnings(&DiagnosticPolicy::ErrorDefaultAction);
  const std::string result = ResolveUri(
      diagnostic, uri, package_map, root_dir);
  EXPECT_NE(result, "");
  return result;
}

// This wraps the "ResolveUri()" device under test to return the error message
// for bad URIs.
std::string ResolveBadUri(
    const std::string& uri,
    const PackageMap& package_map,
    const std::string& root_dir) {
  DiagnosticPolicy diagnostic;
  DiagnosticDetail error;
  diagnostic.SetActionForErrors([&error](const DiagnosticDetail& detail) {
    error = detail;
  });
  // Don't let warnings leak into spdlog; tests should always specifically
  // handle any warnings that appear.
  diagnostic.SetActionForWarnings(&DiagnosticPolicy::ErrorDefaultAction);
  const std::string result = ResolveUri(
      diagnostic, uri, package_map, root_dir);
  EXPECT_EQ(result, "");
  return error.FormatError();
}

// Verifies that the path returned is a normalized path. This is not an
// exhaustive list of all ways a valid path can be unnormalized. Ultimately,
// we're relying on std::filesystem to get the job done and these are just
// indicators (representing common cases) that show it's actually happening.
GTEST_TEST(ResoluveUriUncheckedTest, NormalizedPath) {
  // Use an empty package map.
  PackageMap package_map;

  // We need a valid root directory which respects the bazel sandbox; we'll
  // extract it from a valid resource file.
  const string target_file = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/"
      "sdf/test_model.sdf");
  const string root_dir =
      std::filesystem::path(target_file).parent_path().string();

  // Case: Simple concatenation would produce /fake/root/./file.txt.
  EXPECT_EQ(ResolveGoodUri("./test_model.sdf", package_map, root_dir),
            target_file);

  // Case: Moving up one directory.
  const std::string fake_root = root_dir + "/fake";
  EXPECT_EQ(ResolveGoodUri("../test_model.sdf", package_map, fake_root),
            target_file);

  // Case: Redundant directory dividers.
  EXPECT_EQ(ResolveGoodUri(".//test_model.sdf", package_map, root_dir),
            target_file);
}

// Verifies that ResolveUri() resolves the proper file using the scheme
// 'file://'
GTEST_TEST(ResolveUriTest, TestFile) {
  // Use an empty package map.
  PackageMap package_map;

  // file scheme requires an absolute path.
  const string absolute_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "package_map_test_packages/package_map_test_package_a/"
          "sdf/test_model.sdf");

  // Set the root directory - it will not end up being used in ResolveUri().
  const std::string root_dir = ".";

  const auto uri = std::string("file://") + absolute_path;
  string path = ResolveGoodUri(uri, package_map, root_dir);
  EXPECT_EQ(path, absolute_path);
}

// Verifies that ResolveUri() resolves to the proper file using no scheme and
// an absolute path.
GTEST_TEST(ResolveUriTest, TestAbsolutePath) {
  // Use an empty package map.
  PackageMap package_map;

  const string absolute_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "package_map_test_packages/package_map_test_package_a/"
          "sdf/test_model.sdf");

  // Set the root directory.
  const std::string root_dir = "/";

  string path = ResolveGoodUri(absolute_path, package_map, root_dir);
  EXPECT_EQ(path, absolute_path);
}

// Verifies that ResolveUri() resolves to the proper file using no scheme and
// a relative path.
GTEST_TEST(ResolveUriTest, TestRelativePath) {
  // Use an empty package map.
  PackageMap package_map;

  const std::string relative_path = "package_map_test_packages/"
      "package_map_test_package_a/sdf/test_model.sdf";

  // Set the root directory.
  const std::string root_dir = "multibody/parsing/test/";

  ResolveGoodUri(relative_path, package_map, root_dir);
}

// Verifies that ResolveUri() does not resolve relative paths when root_dir is
// unset.
GTEST_TEST(ResolveUriTest, TestNoRoot) {
  // This is a relative path that _does_ exist, so could be accidentally found
  // by ResolveUri if it had a bug treating an empty root_dir to indicate cwd
  // or similar.  (We confirm that the path is valid by finding it as a
  // resource, but ignoring the returned absolute path).
  const string rel_path =
      "multibody/parsing/test/"
          "package_map_test_packages/package_map_test_package_a/"
          "sdf/test_model.sdf";
  unused(FindResourceOrThrow("drake/" + rel_path));
  const PackageMap package_map;
  const std::string root_dir;
  EXPECT_THAT(
      ResolveBadUri(rel_path, package_map, root_dir),
      ::testing::MatchesRegex(
          ".*URI 'multibody/parsing/.*invalid when parsing a string.*"));
}

// Verifies that ResolveUri() resolves to the proper file using the scheme
// 'model://'
GTEST_TEST(ResolveUriTest, TestModel) {
  const string sdf_file_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "package_map_test_packages/package_map_test_package_a/"
          "sdf/test_model.sdf");

  // Create the package map.
  PackageMap package_map;
  package_map.AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "package_map_test_packages/package_map_test_package_a/"
          "package.xml"));

  // Set the root directory - it will not end up being used in ResolveUri().
  const std::string root_dir = "/no/such/root";

  // Create the URI.
  const string uri_model =
      "model://package_map_test_package_a/sdf/test_model.sdf";
  EXPECT_EQ(ResolveGoodUri(uri_model, package_map, root_dir), sdf_file_name);

  // Create another URI using "package":
  const string uri_package =
      "package://package_map_test_package_a/sdf/test_model.sdf";
  EXPECT_EQ(ResolveGoodUri(uri_package, package_map, root_dir), sdf_file_name);
}

// Verifies that ResolveUri() chokes on an unsupported scheme (like http://)
GTEST_TEST(ResolveUriTest, TestUnsupported) {
  PackageMap package_map;
  const std::string root_dir = ".";
  const string uri = "http://localhost/filename.sdf";
  EXPECT_THAT(ResolveBadUri(uri, package_map, root_dir),
              ::testing::MatchesRegex(".*unsupported scheme.*"));
}

// Verifies that deprecation warnings are produced when accessing deprecated
// package names. We need to cover the case where the package.xml specifies
// a detail message, and the case where there is no detail.
GTEST_TEST(ResolveUriTest, DeprecatedPackage) {
  PackageMap package_map;
  package_map.Add("foo", ".");
  package_map.Add("bar", ".");
  package_map.SetDeprecated("foo", "Stop using foo");
  package_map.SetDeprecated("bar", "");

  // Capture warning messages.
  DiagnosticPolicy diagnostic;
  DiagnosticDetail warning;
  diagnostic.SetActionForWarnings([&warning](const DiagnosticDetail& detail) {
    warning = detail;
  });

  // Check that we get a detailed warning.
  std::string result = ResolveUri(
      diagnostic, "package://foo/multibody", package_map, "");
  EXPECT_EQ(result, "multibody");
  EXPECT_THAT(warning.message, ::testing::MatchesRegex(
      ".*package://foo/multibody.*is deprecated.*Stop using foo.*"));

  // Check that we get a basic warning.
  warning = {};
  result = ResolveUri(
      diagnostic, "package://bar/multibody", package_map, "");
  EXPECT_EQ(result, "multibody");
  EXPECT_THAT(warning.message, ::testing::MatchesRegex(
      ".*package://bar/multibody.*is deprecated.*"));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
