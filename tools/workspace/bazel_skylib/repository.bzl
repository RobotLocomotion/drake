load("//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        upgrade_advice = """
        The commit (version) and sha256 here should be identical to the
        bazel_skylib commit listed in
        drake/tools/install/bazel/test/drake_bazel_installed_test.py.
        """,
        commit = "1.4.2",
        sha256 = "de9d2cedea7103d20c93a5cc7763099728206bd5088342d0009315913a592cc0",  # noqa
        mirrors = mirrors,
    )
