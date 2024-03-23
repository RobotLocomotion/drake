load("//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        commit = "1.5.0",
        sha256 = "118e313990135890ee4cc8504e32929844f9578804a1b2f571d69b1dd080cfb8",  # noqa
        mirrors = mirrors,
    )
