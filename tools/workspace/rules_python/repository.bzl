load("//tools/workspace:github.bzl", "github_archive")
load(
    "//third_party:com_github_bazelbuild_rules_python/internal_config_repo.bzl",  # noqa
    "internal_config_repo",
)

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
        commit = "0.26.0",
        sha256 = "9d04041ac92a0985e344235f5d946f71ac543f1b1565f2cdbc9a2aaee8adf55b",  # noqa
        patches = [
            ":patches/internal_config_repo.patch",
        ],
        mirrors = mirrors,
    )
    internal_config_repo(name = name + "_internal")
