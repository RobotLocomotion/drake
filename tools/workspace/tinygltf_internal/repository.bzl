load("@drake//tools/workspace:github.bzl", "github_archive")

def tinygltf_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "syoyo/tinygltf",
        commit = "e0b393c6958c0a7cbe134a240fad7915aae53db3",
        sha256 = "8e9977178f948c9267897194efc3c920580fb64b353e10e98ce14a797807c791",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/function_pointer.patch",
            ":patches/json.patch",
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
