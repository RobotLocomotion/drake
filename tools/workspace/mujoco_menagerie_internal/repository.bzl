load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "dec370da8895f4a04c20bae57a52f3bd16148ee6",
        sha256 = "79641f12d0431aac351b1c0ab206c378f1999cc1c691f193bae8b0b70833c5a7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
