load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "af46387b0c571d5a4f871d99b266001550a1377a",
        sha256 = "daadbca3ea367642f8d9c2fcf1d20df0184dce66e232310af3d263c892a2cc63",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
