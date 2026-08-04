load("//tools/workspace:github.bzl", "github_archive")

def suitesparse_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DrTimothyAldenDavis/SuiteSparse",
        upgrade_type = "release",
        commit = "v7.12.3.paru",
        sha256 = "99450d2a026f1fc1371bb8267675db59640d1f72b8e1ef3a1dc2449dedd62c7c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
