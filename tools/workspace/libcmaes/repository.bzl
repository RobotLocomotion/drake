load("@drake//tools/workspace:github.bzl", "github_archive")

def libcmaes_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "CMA-ES/libcmaes",
        upgrade_advice = """
        TODO(jwnimmer-tri) We use an untagged commit, in order to use the
        Apache-2.0 license. Any time we upgrade this to a newer commit, we
        should check if there is an official version number yet that we
        could use (i.e., newer than v0.10).
        """,
        commit = "3b171566b1f3803afe1f0f117a98cc14f4d6d331",
        sha256 = "c02a9eaac1537f00ad81a80fd7d0d87a24f831cf20a8449cd6caac794d173682",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
