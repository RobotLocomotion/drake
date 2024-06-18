load(
    "//third_party:com_github_bazelbuild_rules_python/internal_config_repo.bzl",  # noqa
    "internal_config_repo",
)
load("//tools/workspace:github.bzl", "github_archive")

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/rules_python",  # License: Apache-2.0,
        commit = "0.33.0",
        sha256 = "dc6e2756130fafb90273587003659cadd1a2dfef3f6464c227794cdc01ebf70e",  # noqa
        patches = [
            ":patches/internal_config_repo.patch",
        ],
        mirrors = mirrors,
    )
    internal_config_repo(name = name + "_internal")
