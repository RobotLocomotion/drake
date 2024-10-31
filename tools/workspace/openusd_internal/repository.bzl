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
        commit = "v24.11",
        sha256 = "1ab3dc387c8ce8c650d66143c86fb44ed6fe5c9638522bcb3226e85a401c4f69",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/00_work_thread_limits_namespace.patch",
            ":patches/upstream/cmake_gf.patch",
            ":patches/upstream/cmake_rapidjson.patch",
            ":patches/upstream/cmake_usd_usd_shared.patch",
            ":patches/upstream/pegtl_namespace.patch",
            ":patches/upstream/usd_utils_include.patch",
            ":patches/dlopen_forbidden.patch",
            ":patches/namespace.patch",
            ":patches/nanocolor_namespace.patch",
            ":patches/no_gnu_ext.patch",
            ":patches/pegtl_workaround.patch",
            ":patches/weakptrfacade_cxx20.patch",
        ],
        mirrors = mirrors,
    )
