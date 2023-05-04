load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "bca2719a11e688b85ce9af21dcb156f3d8b918bc",
        sha256 = "bd0a1df736b129d65841e910f3dd2350b122fdadaddeb3a123a768c58e45464c",  # noqa
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
