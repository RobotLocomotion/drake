# -*- python -*-

# @see bazelbuild/bazel#3493 for needing `@drake//` when loading `install`.
load("@drake//tools/install:install.bzl", "install")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
)
load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_library",
    "drake_py_test",
)
load("//tools/skylark:6996.bzl", "adjust_label_for_drake_hoist")

_PY_VERSION = "2.7"

# TODO(eric.cousineau): Consider making a `PybindProvider`, to sort
# out dependencies, sources, etc, and simplify installation
# dependencies.

def _drake_pybind_cc_binary(
        name,
        srcs = [],
        copts = [],
        deps = [],
        visibility = None,
        testonly = None):
    """Declares a pybind11 shared library.

    The defines the library with the given name and srcs.
    The libdrake.so library and its headers are already automatically
    depended-on by this rule.
    """
    # TODO(eric.cousineau): Ensure `deps` is header-only, if this code is to
    # live longer.
    drake_cc_binary(
        name = name,
        # This is how you tell Bazel to link in a shared library.
        srcs = srcs + ["//tools/install/libdrake:libdrake.so"],
        # These copts are per pybind11 deficiencies.
        copts = [
            "-Wno-#warnings",
            "-Wno-cpp",
            "-Wno-unknown-warning-option",
        ] + copts,
        # This is how you tell Bazel to create a shared library.
        linkshared = 1,
        linkstatic = 1,
        # For all pydrake_foo.so, always link to Drake and pybind11.
        deps = [
            # Even though "libdrake.so" appears in srcs above, we have to list
            # :drake_shared_library here in order to get its headers onto the
            # include path, and its prerequisite *.so's onto LD_LIBRARY_PATH.
            "//tools/install/libdrake:drake_shared_library",
            "@pybind11",
            # TODO(jwnimmer-tri) We should be getting stx header path from
            # :drake_shared_library, but that isn't working yet.
            "@stx",
        ] + deps,
        testonly = testonly,
        visibility = visibility,
    )

def drake_pybind_library(
        name,
        cc_srcs = [],
        cc_deps = [],
        copts = [],
        cc_so_name = None,
        package_info = None,
        py_srcs = [],
        py_deps = [],
        py_imports = [],
        add_install = True,
        visibility = None,
        testonly = None):
    """Declares a pybind11 library with C++ and Python portions.

    @param cc_srcs
        C++ source files.
    @param cc_deps (optional)
        C++ dependencies.
        At present, these should be libraries that will not cause ODR
        conflicts (generally, header-only).
        By default, this includes `pydrake_pybind`.
    @param cc_so_name (optional)
        Shared object name. By default, this is `_${name}`, so that the C++
        code can be then imported in a more controlled fashion in Python.
        If overridden, this could be the public interface exposed to the user.
    @param package_info
        This should be the result of `get_pybind_package_info` called from the
        current package. This dictates how `PYTHONPATH` is configured, and
        where the modules will be installed.
    @param py_srcs (optional)
        Python sources.
    @param py_deps (optional)
        Python dependencies.
    @param py_imports (optional)
        Additional Python import directories.
    @param add_install (optional)
        Add install targets.
    """
    if package_info == None:
        fail("`package_info` must be supplied.")
    py_name = name
    if not cc_so_name:
        cc_so_name = "_" + name
    # TODO(eric.cousineau): See if we can keep non-`*.so` target name, but
    # output a *.so, so that the target name is similar to what is provided.
    cc_so_name += ".so"
    install_name = name + "_install"
    # Add C++ shared library.
    _drake_pybind_cc_binary(
        name = cc_so_name,
        srcs = cc_srcs,
        deps = cc_deps + [
            "//bindings/pydrake:pydrake_pybind",
        ],
        testonly = testonly,
        visibility = visibility,
    )
    # Add Python library.
    drake_py_library(
        name = py_name,
        data = [cc_so_name],
        srcs = py_srcs,
        deps = py_deps,
        imports = package_info.py_imports + py_imports,
        testonly = testonly,
        visibility = visibility,
    )
    # Add installation target for C++ and C++ bits.
    if add_install:
        install(
            name = install_name,
            targets = [
                py_name,
                cc_so_name,
            ],
            py_dest = package_info.py_dest,
            library_dest = package_info.py_dest,
            visibility = visibility,
        )

def get_drake_py_installs(targets):
    """Gets install targets for Python targets / packages that have a sibling
    install target.

    @note This does not check the targets for correctness.
    """
    return [_get_install(target) for target in targets]

def _get_install(target):
    # Gets the install target for a Python target that has a sibling install
    # target.
    if ":" in target:
        # Append suffix to target.
        return target + "_install"
    else:
        # Assume that the package has an ":install" target.
        return target + ":install"

def get_pybind_package_info(base_package, sub_package = None):
    """Gets a package's path relative to a base package, and the sub-package
    name (for installation).

    @param base_package
        Base package, which should be on `PYTHONPATH`.
    @param sub_package
        Package of interest. If `None`, will resolve to the calling package.
    @return struct(
        py_imports,  # Directories to add to `PYTHONPATH` with `py_library`.
        py_dest)  # Installation directory for use with `install()`.
    """
    # Use relative package path, as `py_library` does not like absolute package
    # paths.
    package_info = _get_package_info(base_package, sub_package)
    return struct(
        py_imports = [package_info.base_path_rel],
        py_dest = "lib/python{}/site-packages/{}".format(
            _PY_VERSION, package_info.sub_path_rel))

def _get_package_info(base_package, sub_package = None):
    # TODO(eric.cousineau): Move this to `python.bzl` or somewhere more
    # general?
    base_package = base_package.lstrip('//')
    if sub_package == None:
        sub_package = native.package_name()
    else:
        sub_package = sub_package.lstrip('//')
    base_package_pre = base_package + "/"
    if not sub_package.startswith(base_package_pre):
        fail("Invalid sub_package '{}' (not a child of '{}')"
             .format(sub_package, base_package))
    sub_path_rel = sub_package[len(base_package_pre):]
    # Count the number of pieces.
    num_pieces = len(sub_path_rel.split("/"))
    # Make the number of parent directories.
    base_path_rel = "/".join([".."] * num_pieces)
    return struct(
        # Base package's path relative to sub-package's path.
        base_path_rel = base_path_rel,
        # Sub-package's path relative to base package's path.
        sub_path_rel = sub_path_rel)

def drake_pybind_cc_googletest(
        name,
        cc_srcs = [],
        py_deps = [],
        cc_deps = [],
        args = [],
        visibility = None,
        tags = []):
    """Defines a C++ test (using `pybind`) which has access to Python
    libraries. """
    cc_name = name + "_cc"
    if not cc_srcs:
        cc_srcs = ["test/{}.cc".format(name)]
    drake_cc_googletest(
        name = cc_name,
        srcs = cc_srcs,
        deps = cc_deps + [
            "//bindings/pydrake:pydrake_pybind",
            "//tools/install/libdrake:drake_shared_library",
            "@pybind11",
        ],
        # Add 'manual', because we only want to run it with Python present.
        tags = ["manual"],
        visibility = visibility,
    )

    py_name = name + "_py"
    # Expose as library, to make it easier to expose Bazel environment for
    # external tools.
    drake_py_library(
        name = py_name,
        deps = py_deps,
        testonly = 1,
        visibility = visibility,
    )

    # Use this Python test as the glue for Bazel to expose the appropriate
    # environment for the C++ binary.
    py_main = "//tools/skylark:py_env_runner.py"
    drake_py_test(
        name = name,
        srcs = [py_main],
        main = py_main,
        data = [cc_name],
        args = ["$(location {})".format(cc_name)] + args,
        deps = [py_name],
        tags = tags,
        visibility = visibility,
    )
