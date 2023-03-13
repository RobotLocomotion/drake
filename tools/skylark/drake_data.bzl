def models_filegroup(
        name,
        extra_srcs = [],
        glob_exclude = [],
        visibility = None):
    """Creates a filegroup with the given name, using a Bazel glob() to find
    files typically associated with model data (e.g., `*.urdf` or `*.obj`).

    Models within a `test/` directory or named `test*` are skipped; this macro
    is only intended to locate non-test models. Tests should prefer to declare
    their dependencies without relying on globbing, either by listing the data
    files directly, or by writing out a specific filegroup() with the details.

    Use `extra_srcs` to add more files beyond what's matched by the glob.

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
        srcs = glob_result + extra_srcs,
        visibility = visibility,
    )
