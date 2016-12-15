# TODO(david-german-tri): Add support for OS X.
cc_library(
    name = "lib",
    # In the Gurobi package, libgurobi60.so is just a symlink to
    # libgurobi.so.6.0.5. However, if you use libgurobi.so.6.0.5 in srcs,
    # executables that link this library will be unable to find it at runtime
    # in the Bazel sandbox, because the NEEDED statements in the executable
    # will not square with the RPATH statements. I don't really know why this
    # happens, but I suspect it might be a Bazel bug.
    srcs = ["gurobi-distro/lib/libgurobi60.so"],
    hdrs = [
        "gurobi-distro/include/gurobi_c.h",
        "gurobi-distro/include/gurobi_c++.h",
    ],
    includes = ["gurobi-distro/include"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
