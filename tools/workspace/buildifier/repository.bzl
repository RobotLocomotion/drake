load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v7.1.1",
        attachments = {
            "buildifier-darwin-amd64": "77460d957af7a028bb18adb6dc43fa64b6e00064e473eac8368ab8a703b3a743",  # noqa
            "buildifier-darwin-arm64": "c990f4b03b27d6a0d86ffe93014732a59c18bab0c4f3a4cc5634bd3b1629fce3",  # noqa
            "buildifier-linux-amd64": "54b7f2ce8f22761cfad264416e912056ae9c8645f59eb6583b846b4864a1ee83",  # noqa
            "buildifier-linux-arm64": "1d9af1f6956a439fca2878a2fbf760b97ca5df00f669e5d1953beb0c4607ac71",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
