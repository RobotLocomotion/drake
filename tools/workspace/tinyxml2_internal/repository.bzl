load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyxml2_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "leethomason/tinyxml2",
        commit = "9.0.0",
        sha256 = "cc2f1417c308b1f6acc54f88eb70771a0bf65f76282ce5c40e54cfe52952702c",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/namespace.patch",
        ],
        mirrors = mirrors,
    )
