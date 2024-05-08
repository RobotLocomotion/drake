load("//tools/workspace:github.bzl", "github_archive")

def openusd_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "PixarAnimationStudios/OpenUSD",
        upgrade_advice = """
        After upgrading, you must re-generate the lockfile:
        bazel run //tools/workspace/openusd_internal:upgrade -- --relock
        """,
        commit = "v24.05",
        sha256 = "0352619895588efc8f9d4aa7004c92be4e4fa70e1ccce77e474ce23941c05828",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/cmake_rapidjson.patch",
            ":patches/cmake_usd_usd_shared.patch",
            ":patches/namespace.patch",
            ":patches/no_gnu_ext.patch",
            ":patches/onetbb.patch",
            ":patches/stage_operatoreq_cxx20.patch",
        ],
        mirrors = mirrors,
    )
