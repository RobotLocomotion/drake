#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_cache.h"
#include "drake/common/find_resource.h"
#include "drake/common/scope_exit.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace {

namespace fs = std::filesystem;
using RemoteParams = PackageMap::RemoteParams;

class PackageMapRemoteTest : public ::testing::Test {
 protected:
  /* Creates a mock-up of the ~/.cache/drake/package_map/{sha256} directory
  containing just the package.xml file. Returns the package directory. */
  fs::path PrepopulateCache(std::string_view package_name,
                            std::string_view sha256) {
    const auto try_cache = drake::internal::FindOrCreateCache("package_map");
    DRAKE_DEMAND(try_cache.error.empty());
    const fs::path package_dir = try_cache.abspath / sha256;
    const bool exists = fs::create_directory(package_dir);
    DRAKE_DEMAND(exists);
    std::ofstream xml(package_dir / "package.xml");
    static constexpr char kPattern[] = R"""(
<?xml version="1.0"?>
<package>
  <name>{}</name>
</package>
)""";
    xml << fmt::format(kPattern, package_name);
    return package_dir;
  }
};

// When the cache directory already contains the content-addressable {sha256}
// directory, the map returns that directory without downloading anything.
TEST_F(PackageMapRemoteTest, GetPathPrepopulated) {
  const std::string package_name("some_remote_name");
  const std::string sha256(64u, '0');

  // Adding the remote package doesn't download anything.
  PackageMap dut = PackageMap::MakeEmpty();
  RemoteParams params;
  params.urls.push_back("http://127.0.0.1/missing.zip");
  params.sha256 = sha256;
  dut.AddRemote(package_name, params);

  // Only a call to GetPath kicks off the resolution, and even then nothing is
  // downloaded (the missing URL would have been a 404 error).
  const fs::path package_dir = PrepopulateCache(package_name, sha256);
  const std::string resolved = dut.GetPath(package_name);
  EXPECT_EQ(resolved, package_dir.string());
}

// Returns a valid remote params object, i.e., one that can actually fetch.
RemoteParams MakeGoodParams() {
  RemoteParams params;
  params.urls.push_back(fmt::format(
      "file://{}",
      FindResourceOrThrow("drake/multibody/parsing/test/"
                          "package_map_test_packages/compressed.zip")));
  params.sha256 =
      "b4bdbad313293ca61fe8f4ed1b5579dadadb3a5c08f0a6d06a8e39e5f97f1bd1";
  params.strip_prefix = "compressed_prefix";
  return params;
}

// Cover the sunny-day control flow for fetching a remote zip file.
TEST_F(PackageMapRemoteTest, ActuallyFetch) {
  // Declare a remote package (using a file:// URI).
  PackageMap dut = PackageMap::MakeEmpty();
  const std::string package_name("compressed");
  dut.AddRemote(package_name, MakeGoodParams());

  // Resolve the package, to force a "download" and extract.
  const std::string resolved = dut.GetPath(package_name);

  // Check that the extracted data lives where it should. (Note that here we're
  // checking the README file within the downloaded package, not the metadata
  // {sha256}.README file from the downloader program in the parent directory.)
  const fs::path readme_path = fs::path(resolved) / "README";
  EXPECT_TRUE(fs::is_regular_file(readme_path)) << readme_path;

  // Double-check the file content.
  std::ifstream readme(readme_path);
  std::stringstream buffer;
  buffer << readme.rdbuf();
  EXPECT_EQ(buffer.str(), "This package is empty.\n");
}

// Cannot duplicate another package, even if its local-only.
TEST_F(PackageMapRemoteTest, RejectDuplicates) {
  PackageMap dut = PackageMap::MakeEmpty();
  dut.Add("foo", ".");

  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddRemote("foo", MakeGoodParams()),
                              ".*existing path is local.*");
}

// Invalid params produce exception messages.
TEST_F(PackageMapRemoteTest, InputSanitization) {
  PackageMap dut = PackageMap::MakeEmpty();

  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddRemote("foo", RemoteParams{}),
                              ".*at least one URL.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.AddRemote("foo",
                    RemoteParams{
                        .urls = {"mailfrom:rico@drake.mit.edu"},
                    }),
      ".*unsupported URL.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.AddRemote("foo",
                    RemoteParams{
                        .urls = {"http://127.0.0.1/missing.zip"},
                        .sha256 = "256",
                    }),
      ".*invalid sha256.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.AddRemote("foo",
                    RemoteParams{
                        .urls = {"http://127.0.0.1/missing.zip"},
                        .sha256 = std::string(64u, 'X'),
                    }),
      ".*invalid sha256.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.AddRemote("foo",
                    RemoteParams{
                        .urls = {"http://127.0.0.1/missing.zip"},
                        .sha256 = std::string(64u, '0'),
                        .archive_type = "arj",
                    }),
      ".*unsupported.*type.*arj.*");
}

TEST_F(PackageMapRemoteTest, CouldNotCreateCache) {
  // Create the cache directory (it probably exists already, but *shrug*).
  const char* const test_tmpdir = std::getenv("TEST_TMPDIR");
  DRAKE_DEMAND(test_tmpdir != nullptr);
  const fs::path cache = fs::path(test_tmpdir) / ".cache";
  fs::create_directory(cache);

  // Remove all permissions (most notably, write permission).
  const fs::perms old_perms = fs::status(cache).permissions();
  fs::permissions(cache, fs::perms{});
  ScopeExit guard([&cache, &old_perms]() {
    fs::permissions(cache, old_perms);
  });

  // Add and fetch a remote package. It will fail due to lack of cache.
  PackageMap dut = PackageMap::MakeEmpty();
  dut.AddRemote("foo", MakeGoodParams());
  DRAKE_EXPECT_THROWS_MESSAGE(dut.GetPath("foo"), ".*create.*cache.*");
}

// Merge an unfetched remote package into the the current map.
TEST_F(PackageMapRemoteTest, AddMapMergeUnfetched) {
  PackageMap compressed = PackageMap::MakeEmpty();
  compressed.AddRemote("compressed", MakeGoodParams());

  // Merge a remote package into the dut.
  PackageMap dut = PackageMap::MakeEmpty();
  dut.AddMap(compressed);
  EXPECT_THAT(dut.GetPackageNames(), testing::ElementsAre("compressed"));

  // Fetch it and make sure it came through.
  const fs::path readme = fs::path(dut.GetPath("compressed")) / "README";
  EXPECT_TRUE(fs::is_regular_file(readme)) << readme;
}

// Merge a fetched remote package into the the current map.
TEST_F(PackageMapRemoteTest, AddMapMergeFetched) {
  PackageMap compressed = PackageMap::MakeEmpty();
  compressed.AddRemote("compressed", MakeGoodParams());
  compressed.GetPath("compressed");

  // Merge the fetched package into the dut.
  PackageMap dut = PackageMap::MakeEmpty();
  dut.AddMap(compressed);
  EXPECT_THAT(dut.GetPackageNames(), testing::ElementsAre("compressed"));
  const fs::path readme = fs::path(dut.GetPath("compressed")) / "README";
  EXPECT_TRUE(fs::is_regular_file(readme)) << readme;
}

// Merging identical remote packages is a no-op.
TEST_F(PackageMapRemoteTest, AddMapDeduplicate) {
  PackageMap dut = PackageMap::MakeEmpty();
  dut.AddRemote("compressed", MakeGoodParams());
  PackageMap other(dut);
  dut.AddMap(other);
  EXPECT_THAT(dut.GetPackageNames(), testing::ElementsAre("compressed"));
}

// Merging differing remote packages is a failure.
TEST_F(PackageMapRemoteTest, AddMapMismatch) {
  RemoteParams params1 = MakeGoodParams();
  RemoteParams params2 = params1;
  params2.urls.push_back(params1.urls.back());

  PackageMap dut = PackageMap::MakeEmpty();
  dut.AddRemote("compressed", params1);
  PackageMap other = PackageMap::MakeEmpty();
  other.AddRemote("compressed", params2);

  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddMap(other), ".*parameters differ.*");
}

}  // namespace
}  // namespace multibody
}  // namespace drake
