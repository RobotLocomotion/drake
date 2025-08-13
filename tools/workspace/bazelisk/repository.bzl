load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def bazelisk_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("bazelisk_repository")
    github_archive(
        name = name,
        repository = "bazelbuild/bazelisk",
        upgrade_advice = """
        When updating, the following additional steps (run in the Drake source
        tree) must also be performed:

        $ bazel build @bazelisk//:*
        $ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
            bazel-drake/external/+internal_repositories+bazelisk/LICENSE \\
            bazel-drake/external/+internal_repositories+bazelisk/bazelisk.py

        Additionally, you must manually update the version number in
          setup/ubuntu/source_distribution/install_bazelisk.sh
        and adjust the expected checksums accordingly.

        To calculate the checksums, download the deb files specifed in
        install_bazelisk.sh and use:
          shasum -a 256 'xxx.deb'

        To fully test, a Linux uprovisioned job must be launched from the
        pull request.
        """,
        commit = "v1.27.0",
        sha256 = "d4abfac1a39876ec1e6c6fa04ec0b62cc4bef174f11d19848bc80dc15ee05261",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
