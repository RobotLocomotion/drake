load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "calderpg/voxelized_geometry_tools",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake.
        """,
        commit = "26bc137d452456b4199b4aa2affed9b54c1674a7",
        sha256 = "d10659f44fc0c4ccd224bd7fffa2687af0ab69541d80808db051bbba6df91c35",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
