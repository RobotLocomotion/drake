load("@drake//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_python",  # License: Apache-2.0,
        upgrade_advice = """
        The commit (version) and sha256 here should be identical to the
        rules_python commit listed in
        drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        """,
        commit = "0.22.0",
        sha256 = "863ba0fa944319f7e3d695711427d9ad80ba92c6edd0b7c7443b84e904689539",  # noqa
        mirrors = mirrors,
    )
