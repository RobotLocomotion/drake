load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v8.0.3",
        attachments = {
            "buildifier-darwin-amd64": "b7a3152cde0b3971b1107f2274afe778c5c154dcdf6c9c669a231e3c004f047e",  # noqa
            "buildifier-darwin-arm64": "674c663f7b5cd03c002f8ca834a8c1c008ccb527a0a2a132d08a7a355883b22d",  # noqa
            "buildifier-linux-amd64": "c969487c1af85e708576c8dfdd0bb4681eae58aad79e68ae48882c70871841b7",  # noqa
            "buildifier-linux-arm64": "bdd9b92e2c65d46affeecaefb54e68d34c272d1f4a8c5b54929a3e92ab78820a",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
