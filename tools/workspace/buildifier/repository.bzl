load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v7.3.1",
        attachments = {
            "buildifier-darwin-amd64": "375f823103d01620aaec20a0c29c6cbca99f4fd0725ae30b93655c6704f44d71",  # noqa
            "buildifier-darwin-arm64": "5a6afc6ac7a09f5455ba0b89bd99d5ae23b4174dc5dc9d6c0ed5ce8caac3f813",  # noqa
            "buildifier-linux-amd64": "5474cc5128a74e806783d54081f581662c4be8ae65022f557e9281ed5dc88009",  # noqa
            "buildifier-linux-arm64": "0bf86c4bfffaf4f08eed77bde5b2082e4ae5039a11e2e8b03984c173c34a561c",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
