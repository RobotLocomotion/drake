load("@drake//tools/workspace:github.bzl", "github_archive")

def tinyobjloader_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "tinyobjloader/tinyobjloader",
        commit = "f2575c576326adadf0872303ec65e9f27ff85ea6",
        sha256 = "938749c75329cc12d7e40ce833ab65d46b6f8550799902e0377605334f9e7e1f",  # noqa
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
