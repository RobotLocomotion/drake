load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.47",
        sha256 = "631a4c58ea6c10c81f160c4b21fa8495b715d251698ebc2552077e8450f30454",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
