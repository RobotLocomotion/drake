load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "f5569db1ffb3b0222663ba38a7a9b3f6a461c469",
        sha256 = "f0a0f5ee4e1c7cc573fba9044c1dbed2f547b3ff058840fc8f83c6b7b79f2391",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
        patches = [
            # We select tinyobjloader's floating-point typedef using a patch
            # file instead of `defines = []` because tinyobjloader is a private
            # dependency of Drake and we don't want the definition to leak into
            # all target that consume Drake.
            ":double_precision.patch",
            # We replace tinyobjloader's implementation of float parsing with a
            # faster call to strtod_l.
            ":faster_float_parsing.patch",
            # If only a diffuse texture is given (map_Kd) tinyobj modulates it
            # to 60% grey. We prefer 100%.
            ":default_texture_color.patch",
        ],
    )
