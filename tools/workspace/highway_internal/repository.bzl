load("//tools/workspace:github.bzl", "github_archive")

def highway_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/highway",
        commit = "1.2.0",
        sha256 = "7e0be78b8318e8bdbf6fa545d2ecb4c90f947df03f7aadc42c1967f019e63343",  # noqa
        patches = [
            ":patches/linkstatic.patch",
            ":patches/target_get_index_inline_always.patch",
            ":patches/target_update_noinline.patch",
        ],
        mirrors = mirrors,
    )
