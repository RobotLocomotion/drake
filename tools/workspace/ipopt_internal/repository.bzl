load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        upgrade_type = "release",
        commit = "releases/3.14.20",
        sha256 = "43bddd6fa793b1694aa94d6129fe3e4a8d452d97a84ef5f2ff721c8047c75605",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/exception_visibility.patch",
        ],
        mirrors = mirrors,
    )
