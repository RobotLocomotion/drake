load("//tools/install:install.bzl", "install")
load("//tools/skylark:cc.bzl", "cc_binary")
load("//tools/skylark:drake_cc.bzl", "drake_cc_binary")
load("//tools/skylark:drake_py.bzl", "drake_py_library", "drake_py_test")
load("//tools/skylark:py.bzl", "py_library")

EXTRA_PYBIND_COPTS = select({
    "@rules_cc//cc/compiler:clang": [
        # GCC and Clang don't always agree / succeed when inferring storage
        # duration (#9600). Workaround it for now.
        "-Wno-unused-lambda-capture",
        # pybind11's operator overloading (e.g., .def(py::self + py::self))
        # spuriously triggers this warning, so we'll suppress it anytime we're
        # compiling pybind11 modules.
        "-Wno-self-assign-overloaded",
    ],
    "//conditions:default": [],
})

def pybind_py_library(
        name,
        cc_srcs = [],
        cc_deps = [],
        cc_copts = [],
        cc_so_name = None,
        cc_binary_rule = cc_binary,
        py_srcs = [],
        py_deps = [],
        py_imports = [],
        py_data = [],
        py_library_rule = py_library,
        **kwargs):
    """Declares a pybind11 Python library with C++ and Python portions.

    @param cc_srcs
        C++ source files.
    @param cc_deps (optional)
        C++ dependencies.
        At present, these should be libraries that will not cause ODR
        conflicts (generally, header-only).
    @param cc_so_name (optional)
        Shared object name. By default, this is `${name}`, so that the C++
        code can be then imported in a more controlled fashion in Python.
        If overridden, this could be the public interface exposed to the user.
    @param py_srcs (optional)
        Python sources.
    @param py_deps (optional)
        Python dependencies.
    @param py_imports (optional)
        Additional Python import directories.
    @param py_data (optional)
        Python data dependencies.
    @return struct(cc_so_target = ..., py_target = ...)
    """
    py_name = name
    if not cc_so_name:
        cc_so_name = name

    # TODO(eric.cousineau): See if we can keep non-`*.so` target name, but
    # output a *.so, so that the target name is similar to what is provided.
    cc_so_target = cc_so_name + ".so"

    # Add C++ shared library.
    cc_binary_rule(
        name = cc_so_target,
        srcs = cc_srcs,
        # This is how you tell Bazel to create a shared library.
        linkshared = 1,
        linkstatic = 1,
        copts = cc_copts + EXTRA_PYBIND_COPTS,
        # Always link to pybind11.
        deps = [
            "@drake//tools/workspace/pybind11",
        ] + cc_deps,
        **kwargs
    )

    # Add Python library.
    py_library_rule(
        name = py_name,
        data = [cc_so_target] + py_data,
        srcs = py_srcs,
        deps = py_deps,
        imports = py_imports,
        **kwargs
    )
    return struct(
        cc_so_target = cc_so_target,
        py_target = py_name,
    )

# TODO(eric.cousineau): Consider making a `PybindProvider`, to sort
# out dependencies, sources, etc, and simplify installation
# dependencies.

# TODO(eric.cousineau): Rename `drake_pybind_library` to
# `drake_pybind_py_library`.

def _check_cc_deps(*, cc_deps, testonly):
    """Fails-fast in case of potential ODR violations."""
    allowed_prefix = [
        # The dep uses a fully-qualified path to somewhere within pydrake.
        "//bindings/generated_docstrings",
        "//bindings/pydrake",
        # The dep is local to pydrake already.
        ":",
        # The dep is a header-only library with no dependencies (unless those
        # dependencies are also header-only).
        "//common:nice_type_name_override_header",
        "//systems/analysis:simulator_python_internal_header",
    ]
    if testonly:
        allowed_prefix.extend([
            # The utilities are not part of libdrake.so, so do not violate ODR.
            "//common/test_utilities",
            # TODO(jwnimmer-tri) This *definitely* violates ODR, but is not
            # quite easy to remove from the one place it's currently used.
            "//common:nice_type_name",
        ])
    for item in (cc_deps or []):
        if any([item.startswith(p) for p in allowed_prefix]):
            continue
        fail("Not allowed to link {} statically".format(item))

def drake_pybind_library(
        name,
        cc_srcs = [],
        cc_deps = [],
        cc_copts = [],
        cc_so_name = None,
        package_info = None,
        py_srcs = [],
        py_deps = [],
        py_imports = [],
        py_data = [],
        add_install = True,
        visibility = None,
        testonly = None):
    """Declares a pybind11 library with C++ and Python portions.

    For parameters `cc_srcs`, `py_srcs`, `py_deps`, `py_imports`, please refer
    to `pybind_py_library`.

    @param cc_deps (optional)
        C++ dependencies.
        At present, these should be libraries that will not cause ODR
        conflicts (generally, header-only).
        By default, this includes `pydrake_pybind` and
        `//:drake_shared_library`.
    @param cc_so_name (optional)
        Shared object name. By default, this is `${name}` (without the `_py`
        suffix if it's present).
    @param package_info
        This should be the result of `get_pybind_package_info` called from the
        current package. This dictates how `PYTHONPATH` is configured, and
        where the modules will be installed.
    @param add_install (optional)
        Add install targets.
    """
    if package_info == None:
        fail("`package_info` must be supplied.")
    _check_cc_deps(cc_deps = cc_deps, testonly = testonly)
    if not cc_so_name:
        if name.endswith("_py"):
            cc_so_name = name[:-3]
        else:
            cc_so_name = name
    install_name = name + "_install"
    targets = pybind_py_library(
        name = name,
        cc_so_name = cc_so_name,
        cc_srcs = cc_srcs,
        cc_deps = cc_deps + [
            "//:drake_shared_library",
            "//bindings/pydrake:pydrake_pybind",
        ],
        cc_copts = cc_copts + [
            # tools/workspace/pybind11/patches/check_signature_infection.patch
            # tweaks our copy of pybind11 to obey this option.
            "-DDRAKE_PYBIND11_CHECK_SIGNATURE_INFECTION=1",
        ],
        cc_binary_rule = drake_cc_binary,
        py_srcs = py_srcs,
        py_deps = py_deps,
        py_imports = package_info.py_imports + py_imports,
        py_data = py_data,
        py_library_rule = drake_py_library,
        testonly = testonly,
        visibility = visibility,
    )

    # Add installation target for C++ and Python bits.
    if add_install:
        install(
            name = install_name,
            targets = [
                targets.cc_so_target,
                targets.py_target,
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
        py_dest = "lib/python@PYTHON_VERSION@/site-packages/{}".format(
            package_info.sub_path_rel,
        ),
    )

def _get_package_info(base_package, sub_package = None):
    # TODO(eric.cousineau): Move this to `python.bzl` or somewhere more
    # general?
    base_package = base_package.lstrip("//")
    if sub_package == None:
        sub_package = native.package_name()
    else:
        sub_package = sub_package.lstrip("//")
    base_package_pre = base_package + "/"
    if not sub_package.startswith(base_package_pre):
        fail("Invalid sub_package '{}' (not a child of '{}')".format(
            # (forced line break)
            sub_package,
            base_package,
        ))
    sub_path_rel = sub_package[len(base_package_pre):]

    # Count the number of pieces.
    num_pieces = len(sub_path_rel.split("/"))

    # Make the number of parent directories.
    base_path_rel = "/".join([".."] * num_pieces)
    return struct(
        # Base package's path relative to sub-package's path.
        base_path_rel = base_path_rel,
        # Sub-package's path relative to base package's path.
        sub_path_rel = sub_path_rel,
    )
