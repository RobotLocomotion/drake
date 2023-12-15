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
        commit = "v1.19.0",
        sha256 = "4c291875707cdd98da87ad34e287c06384436f60cb76e2ac03c32d51f48c96ce",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/pull494.patch",
        ],
        mirrors = mirrors,
    )
