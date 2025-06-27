load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.49",
        sha256 = "e425762fdfb9bb30a5d2da29c0067570e96b5d41d79c659cf0dad861e9df738e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
