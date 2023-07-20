load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "SeanCurtis-TRI/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "0919097b555122cd415390302e4c66e072345f53",
        sha256 = "fe0e2026af3c2510968f0c3952c1d6ae3da8db1ddc2902082c642726b00263e9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
