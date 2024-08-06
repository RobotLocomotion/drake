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
        commit = "v24.08",
        sha256 = "6640bb184bf602c6df14fa4a83af6ac5ae1ab8d1d38cf7bb7decfaa9a7ad5d06",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/00_work_thread_limits_namespace.patch",
            ":patches/cmake_gf.patch",
            ":patches/cmake_rapidjson.patch",
            ":patches/cmake_usd_usd_shared.patch",
            ":patches/dlopen_forbidden.patch",
            ":patches/namespace.patch",
            ":patches/no_gnu_ext.patch",
            ":patches/pegtl_namespace.patch",
            ":patches/usd_sdf_flex_hidden.patch",
            ":patches/weakptrfacade_cxx20.patch",
        ],
        mirrors = mirrors,
    )
