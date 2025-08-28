load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "7f0684e0d2f6c9d75174d1a51e4924b7519119f9",
        sha256 = "82105848fadf0224c105709105e9bab4b6dd27289ac152242cdbfd0be709a364",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
