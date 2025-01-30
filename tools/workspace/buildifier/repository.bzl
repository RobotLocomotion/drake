load("//tools/workspace:github.bzl", "github_release_attachments")

def buildifier_repository(
        name,
        mirrors = None):
    github_release_attachments(
        name = name,
        repository = "bazelbuild/buildtools",
        commit = "v8.0.2",
        attachments = {
            "buildifier-darwin-amd64": "7ee6d4e1de2a57f18013eff62c89fe8db604b327471d792af4c525c15e6218f5",  # noqa
            "buildifier-darwin-arm64": "c6be76f9aa95edeb3cc9d21a747892ca30075513d3d1fdeffbadb64c104b403a",  # noqa
            "buildifier-linux-amd64": "622bf9c483086f96ebb4841c53308ae249cae6eb4ce17c7e438b730d748b2719",  # noqa
            "buildifier-linux-arm64": "ee21828a2172e09f7a03473bfb44cb9170dbc21366e8863a18a904a73ed2a123",  # noqa
        },
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
