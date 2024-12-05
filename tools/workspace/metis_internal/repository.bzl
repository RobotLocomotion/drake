load("//tools/workspace:github.bzl", "github_archive")

def metis_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "KarypisLab/METIS",
        commit = "v5.2.1",
        sha256 = "1a4665b2cd07edc2f734e30d7460afb19c1217c2547c2ac7bf6e1848d50aff7a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/typewidth.patch",
        ],
        mirrors = mirrors,
    )
