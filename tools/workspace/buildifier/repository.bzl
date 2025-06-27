load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v8.2.1",
        attachments = {
            "buildifier-darwin-amd64": "9f8cffceb82f4e6722a32a021cbc9a5344b386b77b9f79ee095c61d087aaea06",  # noqa
            "buildifier-darwin-arm64": "cfab310ae22379e69a3b1810b433c4cd2fc2c8f4a324586dfe4cc199943b8d5a",  # noqa
            "buildifier-linux-amd64": "6ceb7b0ab7cf66fceccc56a027d21d9cc557a7f34af37d2101edb56b92fcfa1a",  # noqa
            "buildifier-linux-arm64": "3baa1cf7eb41d51f462fdd1fff3a6a4d81d757275d05b2dd5f48671284e9a1a5",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
