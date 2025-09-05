load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "c1503a62496b64222c64ff65dd652d461a5b064e",
        sha256 = "82f92b068208412b69eea686f32b7640ce4058398692772567a81f2a9ad4300d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
