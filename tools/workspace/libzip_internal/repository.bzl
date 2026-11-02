load("//tools/workspace:github.bzl", "github_archive")

def libzip_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "nih-at/libzip",
        upgrade_type = "release",
        commit = "v1.12.1",
        sha256 = "eb660b05d6d98742fd0466c416fb7867e9c1c101f00502f1efee0a2c089ad5c4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
