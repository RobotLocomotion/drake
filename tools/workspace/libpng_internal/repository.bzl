load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.58",
        sha256 = "a9d4df463d36a6e5f9c29bd6f4967312d17e996c1854f3511f833924eb1993cf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
