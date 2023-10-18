load("//tools/workspace:github.bzl", "github_archive")

def voxelized_geometry_tools_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/voxelized_geometry_tools",
        upgrade_advice = """
        When updating, ensure that any new unit tests are reflected in
        package.BUILD.bazel and BUILD.bazel in drake.
        """,
        commit = "daa34feecf983a6b10b07add4a51a7e1c499b913",
        sha256 = "ba62cb041ea05239196fd032bfde02e5cd2db2ad64d53dc6610218f8d8080742",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
