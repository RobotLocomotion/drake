#include <fstream>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_cache.h"
#include "drake/common/find_resource.h"
#include "drake/common/scope_exit.h"
#include "drake/common/sha256.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace {

namespace fs = std::filesystem;
using RemoteParams = PackageMap::RemoteParams;

class PackageMapRemoteTest : public ::testing::Test {
 protected:
  /* Creates a mock-up of the cached package download directory containing just
  the package.xml file. Returns the package directory. */
  fs::path PrepopulateCache(std::string_view package_name,
                            std::string_view sha256,
                            std::string_view strip_prefix) {
    const auto try_cache = drake::internal::FindOrCreateCache("package_map");
    DRAKE_DEMAND(try_cache.error.empty());
    const fs::path package_dir =
        try_cache.abspath /
        fmt::format("{}-{}", sha256,
                    Sha256::Checksum(strip_prefix).to_string());
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
  const fs::path package_dir = PrepopulateCache(package_name, sha256, "");

  // Adding the remote package doesn't download anything.
  // (The DRAKE_ALLOW_NETWORK governor would fail if we tried to download.)
  PackageMap dut = PackageMap::MakeEmpty();
  RemoteParams params;
  params.urls.push_back("http://127.0.0.1/missing.zip");
  params.sha256 = sha256;
  dut.AddRemote(package_name, params);

  // Calling GetPath doesn't download, either.
  EXPECT_EQ(dut.GetPath(package_name), package_dir.string());
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
  EXPECT_EQ(ReadFileOrThrow(readme_path), "This package is empty.\n");
}

// Fetch a remote zip file, then again with a different strip_prefix.
TEST_F(PackageMapRemoteTest, FetchWithDifferentStrip) {
  PackageMap dut = PackageMap::MakeEmpty();

  auto foo_params = MakeGoodParams();
  dut.AddRemote("foo", foo_params);

  auto bar_params = MakeGoodParams();
  const std::string old_bar_prefix = std::move(bar_params.strip_prefix.value());
  dut.AddRemote("bar", bar_params);

  // Resolve 'foo' and then 'bar'.
  const auto foo = fs::path(dut.GetPath("foo"));
  const auto bar = fs::path(dut.GetPath("bar"));

  // Check that the extracted data lives where it should. (Note that here we're
  // checking the README file within the downloaded package, not the metadata
  // {sha256}.README file from the downloader program in the parent directory.)
  EXPECT_TRUE(fs::is_regular_file(foo / "README"));
  EXPECT_TRUE(fs::is_regular_file(bar / old_bar_prefix / "README"));
}

// When DRAKE_ALLOW_NETWORK denies package_map, only file:// URLs are allowed
// and others (like http://) are quietly ignored. (Note that our Bazel config
// runs unit tests as "denied by default"; our BUILD rule doesn't need any
// special magic in support of this test case.)
TEST_F(PackageMapRemoteTest, GetPathEnvironmentDenied) {
  // Prepare a remote package that specifies both http and file.
  PackageMap::RemoteParams params = MakeGoodParams();
  const std::string file_url = params.urls.front();
  const std::string http_url = "http://127.0.0.1/missing.zip";
  params.urls = {http_url, file_url};

  // Add and fetch it.
  PackageMap dut = PackageMap::MakeEmpty();
  EXPECT_NO_THROW(dut.AddRemote("ok", params));
  EXPECT_NO_THROW(dut.GetPath("ok"));

  // Without the file url to fall back on, http-only is an error (when network
  // is denied). Note that we need to clear the sha256 to avoid caching. Only
  // actually hitting the network is denied.
  params.urls = {http_url};
  params.sha256 = std::string(64u, 'f');
  dut = PackageMap::MakeEmpty();
  DRAKE_EXPECT_THROWS_MESSAGE(dut.AddRemote("bad", params),
                              ".*DRAKE_ALLOW_NETWORK.*");
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

TEST_F(PackageMapRemoteTest, CouldNotFetch) {
  // Declare a remote package with a well-formed but incorrect checksum.
  auto params = MakeGoodParams();
  params.sha256.front() = 'f';
  PackageMap dut = PackageMap::MakeEmpty();
  const std::string package_name("compressed");
  dut.AddRemote(package_name, params);

  // Fetching the package throws.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.GetPath(package_name),
                              ".*downloader.*error[^]*Checksum[^]*");

  // An incomplete download should not corrupt the cache dir. Trying again
  // should still fail.
  DRAKE_EXPECT_THROWS_MESSAGE(dut.GetPath(package_name),
                              ".*downloader.*error[^]*Checksum[^]*");
}

// Merge an unfetched remote package into the the current map. Our purpose here
// is a sanity check that nothing crashes when copying an unfetched PackageData.
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

// Merge a fetched remote package into the the current map. Our purpose here is
// a sanity check that nothing crashes when copying a fetched PackageData, but
// also to confirm that the presence of the known local path does not prevent us
// from preserving the remote params for later use.
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

  // Confirm the remote data (URLs) was still preserved, even though we already
  // had a fetched path in available during the AddMap.
  EXPECT_THAT(fmt::to_string(fmt_streamed(dut)),
              testing::HasSubstr("package_map_test_packages/compressed.zip"));
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

// Sanity check the `package://drake_models/...` defaults.
TEST_F(PackageMapRemoteTest, DrakeModelsDefaults) {
  // Check that the package exists by default.
  std::string drake_models_path;
  EXPECT_NO_THROW(drake_models_path = PackageMap().GetPath("drake_models"));
  EXPECT_TRUE(fs::is_directory(drake_models_path)) << drake_models_path;

  // Check its remote fetching configuration (using an internal API, because we
  // don't actually want to fetch it from the network during a unit test).
  const RemoteParams params = internal::GetDrakeModelsRemoteParams();
  EXPECT_GE(params.urls.size(), 1);
  EXPECT_THAT(params.urls, testing::Each(testing::StartsWith("https://")));
  EXPECT_EQ(params.sha256.size(), 64);
  EXPECT_EQ(params.archive_type, std::nullopt);
  EXPECT_THAT(params.strip_prefix,
              testing::Optional(testing::StartsWith("models-")));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
