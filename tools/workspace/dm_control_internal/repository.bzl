load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "f2f0e2333d8bd82c0b6ba83628fe44c2bcc94ef5",
        sha256 = "aa5d951089bf258c3799e38b0d68c7cdaadc351988e062e32f04c0e58353d6d0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
