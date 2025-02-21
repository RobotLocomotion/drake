load(
    "//third_party:com_github_bazelbuild_rules_python/internal_config_repo.bzl",  # noqa
    "internal_config_repo",
)
load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

# (Internal use only) Creates a @rules_python_drake_constants repository that
# communicates whether @rules_python is using Drake's pinned version or Bazel's
# vendored copy.
def _rules_python_drake_constants_repository_impl(repo_ctx):
    constants_json = repo_ctx.attr.constants_json
    repo_ctx.file("BUILD.bazel", "")
    repo_ctx.file("defs.bzl", "CONSTANTS = {}".format(constants_json))

_rules_python_drake_constants_repository = repository_rule(
    implementation = _rules_python_drake_constants_repository_impl,
    attrs = {
        "constants_json": attr.string(),
    },
)

# Note that we do NOT install a LICENSE file as part of the Drake install
# because this repository is required only when building and testing with
# Bazel.

def rules_python_repository(
        name,
        mirrors = None,
        _constants_only = False,
        _is_drake_self_call = False):
    """Declares the @rules_python repository (if necessary) as well as the
    @rules_python_drake_constants repository (always).

    When `_constants_only` is true, the @rules_python repository will NOT be
    declared; only @rules_python_drake_constants is declared. (In practice,
    this happens when "rules_python" is listed in the `repository_excludes`
    for `default.bzl`, e.g., when bzlmod is enabled.)

    Even when _constants_only is false, the @rules_python repository still
    might not be declared according to the heuristic described below.
    """
    if not _is_drake_self_call:
        print_warning("rules_python_repository")

    # For Bazel versions < 8, we pin our own particular copy of rules_python,
    # because the bazel's default (vendored) version is somewhat stale. For
    # Bazel versions >= 8, we'll use Bazel's vendored copy of rules_python when
    # running in WORKSPACE mode. Our minimum version (per WORKSPACE) is 7.1 so
    # we can use a string match. When running in MODULE (bzlmod) mode, the
    # _constants_only will be True (so we will NOT pin).
    use_drake_rules_python_pin = (native.bazel_version[0:2] == "7." and
                                  not _constants_only)
    _rules_python_drake_constants_repository(
        name = name + "_drake_constants",
        constants_json = json.encode({
            "USE_DRAKE_PIN": 1 if use_drake_rules_python_pin else 0,
        }),
    )
    if not use_drake_rules_python_pin:
        return
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
