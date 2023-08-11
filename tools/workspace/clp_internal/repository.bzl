load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def clp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Clp",
        commit = "releases/1.17.8",
        sha256 = "f9931b5ba44f0daf445c6b48fc2c250dc12e667e59ace8ea7b025f158fe31556",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/missing_include.patch",
        ],
        mirrors = mirrors,
    )
