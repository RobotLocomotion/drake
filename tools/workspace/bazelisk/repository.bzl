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
        commit = "v1.24.1",
        sha256 = "c7a44600ae88732fd75d8cbd1d58efe69610e41540566ff4102c5e3b96e497a7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/pull494.patch",
        ],
        mirrors = mirrors,
    )
