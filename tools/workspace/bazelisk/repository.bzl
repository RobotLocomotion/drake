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

        Additionally, you must manually update the version number in
          setup/ubuntu/source_distribution/install_bazelisk.sh
        and adjust the expected checksums accordingly.
        """,
        commit = "v1.22.1",
        sha256 = "64b584d1019d54cde34123d8da06c718c7a7c591f9fd49a29dccb825b9e95e8c",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/pull494.patch",
        ],
        mirrors = mirrors,
    )
