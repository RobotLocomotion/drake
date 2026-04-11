load("//tools/workspace:github.bzl", "github_archive")

def gklib_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "KarypisLab/GKlib",
        commit = "e2856c2f595b153ca1ce9258c5301dbabc4f39f5",
        sha256 = "ece01338c55412f085910968832289fb08e6761f6ce7b94755477077ce449155",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
