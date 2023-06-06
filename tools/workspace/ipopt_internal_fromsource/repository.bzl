load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def ipopt_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        commit = "releases/3.14.12",
        sha256 = "6b06cd6280d5ca52fc97ca95adaaddd43529e6e8637c274e21ee1072c3b4577f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
