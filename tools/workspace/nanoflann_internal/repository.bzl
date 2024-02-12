load("//tools/workspace:github.bzl", "github_archive")

def nanoflann_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        commit = "v1.5.4",
        sha256 = "a7f64d0bdff42614c561e52680b16de46c0edac9719f21f935c5e1f8b0654afc",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
