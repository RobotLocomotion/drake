load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v7.1.2",
        attachments = {
            "buildifier-darwin-amd64": "687c49c318fb655970cf716eed3c7bfc9caeea4f2931a2fd36593c458de0c537",  # noqa
            "buildifier-darwin-arm64": "d0909b645496608fd6dfc67f95d9d3b01d90736d7b8c8ec41e802cb0b7ceae7c",  # noqa
            "buildifier-linux-amd64": "28285fe7e39ed23dc1a3a525dfcdccbc96c0034ff1d4277905d2672a71b38f13",  # noqa
            "buildifier-linux-arm64": "c22a44eee37b8927167ee6ee67573303f4e31171e7ec3a8ea021a6a660040437",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
