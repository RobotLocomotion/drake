load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When updating, the following additional steps (run in the Drake source
        tree) must also be performed:

        $ bazel build @bazelisk//:*
        $ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
            bazel-drake/external/bazelisk/LICENSE \\
            bazel-drake/external/bazelisk/bazelisk.py
        """,
        commit = "v1.21.0",
        sha256 = "0b7b5b74cb5d79ba814b4413e59adb826f3891e1b14dfd1485eae2078f531253",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/pull494.patch",
        ],
        mirrors = mirrors,
    )
