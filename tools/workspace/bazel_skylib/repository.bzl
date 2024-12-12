load("//tools/workspace:github.bzl", "github_archive")

def bazel_skylib_repository(name, mirrors = None):
    github_archive(
        name = name,
        repository = "bazelbuild/bazel-skylib",
        upgrade_advice = """
        When updating, you must also manually propagate to the new version
        number into the MODULE.bazel file (at the top level of Drake).
        """,
        commit = "1.7.1",
        sha256 = "e3fea03ff75a9821e84199466799ba560dbaebb299c655b5307f4df1e5970696",  # noqa
        mirrors = mirrors,
    )
