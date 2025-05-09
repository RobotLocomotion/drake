load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v8.2.0",
        attachments = {
            "buildifier-darwin-amd64": "309b3c3bfcc4b1533d5f7f796adbd266235cfb6f01450f3e37423527d209a309",  # noqa
            "buildifier-darwin-arm64": "e08381a3ed1d59c0a17d1cee1d4e7684c6ce1fc3b5cfa1bd92a5fe978b38b47d",  # noqa
            "buildifier-linux-amd64": "3e79e6c0401b5f36f8df4dfc686127255d25c7eddc9599b8779b97b7ef4cdda7",  # noqa
            "buildifier-linux-arm64": "c624a833bfa64d3a457ef0235eef0dbda03694768aab33f717a7ffd3f803d272",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
