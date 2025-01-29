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
        commit = "v25.02",
        sha256 = "e06f67522cc4ff1a3ce99bc28075b049a15869792b41d33b0d4387fb22122cfe",  # noqa
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
