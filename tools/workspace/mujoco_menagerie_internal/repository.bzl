load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        upgrade_type = "commit",
        commit = "71f066ad0be9cd271f7ed58c030243ef157af9f4",
        sha256 = "000b9f51abb404efb1de2b88b3c738674c472a85b6c4143168859abc4c98d423",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
