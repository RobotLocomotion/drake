load("@drake//tools/workspace:github.bzl", "github_archive")

def clarabel_cpp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/Clarabel.cpp",
        commit = "dc5ed49e0ae32c45e22fffc931c6bee75fb2974d",
        sha256 = "5f377c2b1794b29070f67d487bd5d59403e5143e3122c3eecd5744890613ef3e",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/clarabel_version.patch",
        ],
        mirrors = mirrors,
    )
