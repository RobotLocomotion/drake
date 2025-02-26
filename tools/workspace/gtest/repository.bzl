load("//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "v1.16.0",
        sha256 = "78c676fc63881529bf97bf9d45948d905a66833fbfa5318ea2cd7478cb98f399",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/add_printers.patch",
        ],
        mirrors = mirrors,
    )
