load("//tools/workspace:github.bzl", "github_archive")

def libzip_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nih-at/libzip",
        commit = "v1.11.4",
        sha256 = "4844595615e2436e3cf1ed46a1b260fbdaf8b8fa8f2b594e8b5d8162c696b8b2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
