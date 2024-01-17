load("//tools/workspace:github.bzl", "github_archive")

def expat_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "libexpat/libexpat",
        commit = "be47f6d5e871382dc2ab783a9df416ec4370074e",
        sha256 = "f8d003d61297773c3dbe84b8efc3e6ca1721cf12a1815e378b4a081b9b97eb57",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
