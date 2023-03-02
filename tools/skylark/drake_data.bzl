# -*- python -*-

def models_filegroup(
        name,
        testonly = False,
        extra_srcs = [],
        glob_exclude = [],
        visibility = None):
    """Creates a filegroup with the given name, using a Bazel glob() to find
    files typically associated with model data (e.g., `*.urdf` or `*.obj`).

    Uuse `extra_srcs` to add more files beyond what's matched by the glob.

    Use `glob_exclude` to exclude patterns that would otherwise be found.
    """
    models_extensions = [
        "csv",
        "dae",
        "jpg",
        "json",
        "mtl",
        "obj",
        "png",
        "sdf",
        "stl",
        "urdf",
        "vtm",
        "vtp",
        "xml",
    ]
    include = [
        "**/*.{}".format(x)
        for x in models_extensions
    ]
    exclude = glob_exclude + [
        "**/test/*",
        "**/test*",
    ]
    glob_result = native.glob(
        include = include,
        exclude = exclude,
        allow_empty = True,
    )
    if len(glob_result) == 0:
        fail("models_filegroup() did not match any files")
    native.filegroup(
        name = name,
        testonly = testonly,
        srcs = glob_result + extra_srcs,
        visibility = visibility,
    )
