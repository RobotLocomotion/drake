load("//tools/workspace:github.bzl", "github_archive")

def bazelisk_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When updating, the following additional steps (run in the Drake source
        tree) must also be performed:

        $ bazel build @bazelisk_internal//:*
        $ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
            bazel-drake/external/+internal_repositories+bazelisk_internal/LICENSE \\
            bazel-drake/external/+internal_repositories+bazelisk_internal/bazelisk.py

        Additionally, you must manually update the bazelisk version number in
          setup/ubuntu/install_prereqs.sh
        and adjust the expected checksums accordingly.

        To calculate the checksums, download the deb files specifed in
        install_prereqs.sh and use:
          shasum -a 256 'xxx.deb'

        To fully test, a Linux uprovisioned job must be launched from the
        pull request.
        """,  # noqa
        upgrade_type = "release",
        commit = "v1.29.0",
        sha256 = "7e4c7b8ade016052e63c1553cb4fbe0c4fe921e1e66913d49eef074ed894e933",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
