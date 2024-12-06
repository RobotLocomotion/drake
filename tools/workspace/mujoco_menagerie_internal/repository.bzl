load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "bd9709b540d58e1dcf417e4ffeffc7d54318280d",
        sha256 = "73e4cecf8c2cd92ebfabdecc365d2dfa8ef3dc77ba6994bf6202cecbfd7fa7d7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
