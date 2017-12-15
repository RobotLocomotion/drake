# -*- python -*-

# @see bazelbuild/bazel#3493 for needing `@drake//` when loading `install`.
load("@drake//tools/install:install.bzl", "install")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
)
load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_library",
)
load("//tools/skylark:6996.bzl", "adjust_label_for_drake_hoist")

_PY_VERSION = "2.7"

# This is the base package to determine which paths should be imported, and
# where the Python components should be installed.
#
# TODO(jwnimmer-tri) Either make this path configurable, or else move this file
# (pybind.bzl) back into the bindings folder directly.
_BASE_PACKAGE = adjust_label_for_drake_hoist("//drake/bindings")[2:]

# TODO(eric.cousineau): Consider making a `PybindProvider`, to sort
# out dependencies, sources, etc, and simplify installation
# dependencies.

def _drake_pybind_cc_binary(
        name,
        srcs = [],
        copts = [],
        deps = [],
        visibility = None):
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
        visibility = visibility,
    )

def drake_pybind_library(
        name,
        cc_srcs = [],
        cc_deps = [],
        copts = [],
        cc_so_name = None,
        py_srcs = [],
        py_deps = [],
        py_imports = [],
        add_install = True,
        visibility = None):
    """Declares a pybind11 library with C++ and Python portions.

    @param cc_srcs
        C++ source files.
    @param cc_deps (optional)
        C++ dependencies.
        At present, these should be header only, as they will violate ODR
        with statically-linked libraries.
    @param cc_so_name (optional)
        Shared object name. By default, this is `_${name}`, so that the C++
        code can be then imported in a more controlled fashion in Python.
        If overridden, this could be the public interface exposed to the user.
    @param py_srcs
        Python sources.
    @param py_deps
        Python dependencies.
    @param py_imports
        Additional Python import directories.
    """
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
        deps = cc_deps,
        visibility = visibility,
    )
    # Get current package's information.
    library_info = _get_child_library_info()
    py_base_rel_path, py_library_install = (
        library_info.rel_path, library_info.sub_package)
    # Add Python library.
    drake_py_library(
        name = py_name,
        data = [cc_so_name],
        srcs = py_srcs,
        deps = py_deps,
        imports = [py_base_rel_path] + py_imports,
        visibility = visibility,
    )
    # Add installation target for C++ and C++ bits.
    if add_install:
        py_dest = get_pybind_library_dest(py_library_install)
        install(
            name = install_name,
            targets = [
                py_name,
                cc_so_name,
            ],
            py_dest = py_dest,
            library_dest = py_dest,
            visibility = visibility,
        )

def get_drake_pybind_installs(targets):
    """Gets install targets for `drake_pybind_library` targets.

    @note This does not check the targets for correctness.
    """
    return [_get_install(target) for target in targets]

def _get_install(target):
    # Gets the install target for a `drake_pybind_library` target.
    if ":" in target:
        # Append suffix to target.
        return target + "_install"
    else:
        # Assume that the package has an ":install" target.
        return target + ":install"

def get_pybind_library_dest(py_library_install = None):
    """Gets Python installation destination for a given package."""
    if py_library_install == None:
        py_library_install = _get_child_library_info().sub_package
    return "lib/python{}/site-packages/{}".format(_PY_VERSION,
                                                  py_library_install)

def _get_child_library_info(package = None, base_package = _BASE_PACKAGE):
    # Gets a package's path relative to a base package, and the sub-package
    # name (for installation).
    # @return struct(rel_path, sub_package)
    if package == None:
        package = native.package_name()
    base_package_pre = base_package + "/"
    if not package.startswith(base_package_pre):
        fail("Invalid package '{}' (not a child of '{}')"
             .format(package, base_package))
    sub_package = package[len(base_package_pre):]
    # Count the number of pieces.
    num_pieces = len(sub_package.split("/"))
    # Make the number of parent directories.
    rel_path = "/".join([".."] * num_pieces)
    return struct(rel_path = rel_path, sub_package = sub_package)
