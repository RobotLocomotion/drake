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
        commit = "0.36.0",
        # Versions of rules_python newer than 0.36.0 are too difficult to adapt
        # to our current WORKSPACE regime, so we'll pin to 0.36.x as the newest
        # version we'll support for in-tree builds of Drake. Once we upgrade to
        # using bzlmod instead, we can remove this pin.
        commit_pin = True,
        sha256 = "ca77768989a7f311186a29747e3e95c936a41dffac779aff6b443db22290d913",  # noqa
        patches = [
            ":patches/internal_config_repo.patch",
        ],
        mirrors = mirrors,
    )
    internal_config_repo(name = name + "_internal")
