load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v7.1.0",
        attachments = {
            "buildifier-darwin-amd64": "7002a1f7e9c41fed24210abddfaed0f46f1abc5730faace03ef9ebdd2b9e125e",  # noqa
            "buildifier-darwin-arm64": "1e6f4959a7b0024f3adda80d92fa84e0193a99d234bb00cb13d900f80c08bd6c",  # noqa
            "buildifier-linux-amd64": "edc9e05da569cb99a287464664a2350e85e96fd591b86f66c1527870ad96c0da",  # noqa
            "buildifier-linux-arm64": "89ace4342e3595e4feda9c1ce205542492a33a481a3d0875a1a83448a73c8714",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
