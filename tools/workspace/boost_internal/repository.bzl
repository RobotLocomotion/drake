load("//tools/workspace:github.bzl", "github_release_attachments")

def boost_internal_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "boostorg/boost",
        commit = "boost-1.85.0",
        # TODO(#20898) The new_release tooling can't handle upgrading all of
        # the attachment paths here, so for now we eschew automated upgrades
        # of an external that's disabled by default.
        commit_pin = True,
        attachments = {
            "boost-1.85.0-cmake.tar.xz": "0a9cc56ceae46986f5f4d43fe0311d90cf6d2fa9028258a95cab49ffdacf92ad",  # noqa
        },
        extract = [
            "boost-1.85.0-cmake.tar.xz",
        ],
        strip_prefix = {
            "boost-1.85.0-cmake.tar.xz": "boost-1.85.0",
        },
        build_file = "@com_github_nelhage_rules_boost_internal//:boost.BUILD",
        mirrors = mirrors,
        repo_mapping = {
            "@com_github_nelhage_rules_boost": "@com_github_nelhage_rules_boost_internal",  # noqa
        },
    )
