load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def bazel_skylib_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("bazel_skylib_repository")
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        upgrade_advice = """
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "1.8.0",
        sha256 = "49902c0cc841c1b35616669bf9452858a2849112ae441b5d70a0701ffbafb3d7",  # noqa
        mirrors = mirrors,
    )
