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
        commit = "v1.20.0",
        sha256 = "3c2303d45562cf7a9bc64ad41b670f38c2634bf8ba5b3acffa2997577955b3e0",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/pull494.patch",
        ],
        mirrors = mirrors,
    )
