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
        commit = "v24.03",
        sha256 = "0724421cff8ae04d0a7108ffa7c104e6ec3f7295418d4d50caaae767e59795ef",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/cmake_rapidjson.patch",
            ":patches/cmake_usd_usd_shared.patch",
            ":patches/no_gnu_ext.patch",
            ":patches/onetbb.patch",
            ":patches/stage_operatoreq_cxx20.patch",
        ],
        mirrors = mirrors,
    )
