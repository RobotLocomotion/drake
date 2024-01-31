load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v6.4.0",
        attachments = {
            "buildifier-darwin-amd64": "eeb47b2de27f60efe549348b183fac24eae80f1479e8b06cac0799c486df5bed",  # noqa
            "buildifier-darwin-arm64": "fa07ba0d20165917ca4cc7609f9b19a8a4392898148b7babdf6bb2a7dd963f05",  # noqa
            "buildifier-linux-amd64": "be63db12899f48600bad94051123b1fd7b5251e7661b9168582ce52396132e92",  # noqa
            "buildifier-linux-arm64": "18540fc10f86190f87485eb86963e603e41fa022f88a2d1b0cf52ff252b5e1dd",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
