load("//tools/workspace:github.bzl", "github_release_attachments")

def boost_internal_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "boostorg/boost",
        commit = "boost-1.84.0",
        attachments = {
            "boost-1.84.0.tar.xz": "2e64e5d79a738d0fa6fb546c6e5c2bd28f88d268a2a080546f74e5ff98f29d0e",  # noqa
        },
        extract = [
            "boost-1.84.0.tar.xz",
        ],
        strip_prefix = {
            "boost-1.84.0.tar.xz": "boost-1.84.0",
        },
        build_file = "@com_github_nelhage_rules_boost_internal//:boost.BUILD",
        mirrors = mirrors,
        repo_mapping = {
            "@com_github_nelhage_rules_boost": "@com_github_nelhage_rules_boost_internal",  # noqa
        },
    )
