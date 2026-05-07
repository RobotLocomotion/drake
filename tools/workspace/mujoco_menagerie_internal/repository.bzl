load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "affef0836947b64cc06c4ab1cbf0152835693374",
        sha256 = "f532624be950782dad09e8b36e4b89d880add2804c5dc2a0eea1e0d5562092d6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
