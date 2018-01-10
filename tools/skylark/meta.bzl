def all_files(subdirs = []):
    # Add all sources visibile to the current package.
    # @note It'd be nice if this could respect *ignore files, but meh.
    # Also, it'd be **super** nice if Bazel not let `**` globs leak into other
    # packages and then error out.
    files = native.glob(["*"])
    for subdir in subdirs:
        files += native.glob([subdir + "/*"])
    native.filegroup(
        name = "all_files",
        srcs = files,
        visibility = ["//visibility:public"],
    )
