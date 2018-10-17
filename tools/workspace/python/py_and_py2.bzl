load(
    "@python//:version.bzl",
    "PYTHON_SITE_PACKAGES_RELPATH",
    "PYTHON_VERSION",
)
load(
    "@python2//:version.bzl",
    PYTHON2_SITE_PACKAGES_RELPATH = "PYTHON_SITE_PACKAGES_RELPATH",
    PYTHON2_VERSION = "PYTHON_VERSION",
)

_BAZEL = struct(
    build_field = "bazel",
    version_field = "py" + PYTHON_VERSION.split(".")[0],
    major = PYTHON_VERSION.split(".")[0],
    site_packages = PYTHON_SITE_PACKAGES_RELPATH,
)
_PY2 = struct(
    build_field = "py2",
    version_field = "py2",
    major = PYTHON2_VERSION.split(".")[0],
    site_packages = PYTHON2_SITE_PACKAGES_RELPATH,
)

# Cannot use `type(x) == str`, etc., but can infer from literals?
_list = type([])
_str = type("")
_struct = type(struct())

def py_select(**kwargs):
    return struct(**kwargs)

def py_and_py2(
        rule,
        **kwargs):
    """
    Provides Bazel Python (2 or 3) and Python2 targets, without duplication or
    conflict. This should be used **sparingly**, only when Python2 and Python3
    support is needed within the same build.

    If Bazel Python is Python2, the Python2 target will be an alias if its name
    is unique.
    Otherwise, Python3 and Python2 targets will be defined.

    Resolving `py_select` statements:
        `py_select(bazel=..., py2=...)` - Switch between Bazel Python and
            Python2 (depending on build target being generated).
        `py_select(py2=..., py3=...)` - Switch between Python 2 and Python 3
            (regardless of build target being generated).
    Strings can also have `@PYTHON_SITE_PACKAGES` be replaced with the correct
    version.

    @param rule
        Starlark rule (e.g. `py_library`, `cc_binary`).
    @param kwargs
        Arguments (which are resolved as mentioned above).
    """
    if _PY2.major != "2":
        fail("@python2 has the wrong major version: {}".format(_PY2))

    # Define Bazel Python target.
    kwargs_py = _format(kwargs, _BAZEL)
    rule(**kwargs_py)

    # Define Python2...
    kwargs_py2 = _format(kwargs, _PY2)
    if _BAZEL.major == "2":
        # Alias.
        if kwargs_py2["name"] != kwargs_py["name"]:
            native.alias(
                name = kwargs_py2["name"],
                actual = kwargs_py["name"],
                visibility = kwargs_py2.get("visibility"),
            )
    else:
        # Target.
        rule(**kwargs_py2)

def _format_str(value, build):
    return value.replace("@PYTHON_SITE_PACKAGES@", build.site_packages)

def _format_scalar(value, build):
    if type(value) == _str:
        return _format_str(value, build)
    elif type(value) == _struct:
        if hasattr(value, "bazel"):
            value = getattr(value, build.build_field)
        else:
            value = getattr(value, build.version_field)
        if type(value) == _str:
            return _format_str(value, build)
        else:
            return [_format_str(x, build) for x in value]

def _format(raw, build):
    out = dict()
    for key, value in raw.items():
        # N.B. `str` and `list` do not work in this comparison.
        if type(value) in [_str, _struct]:
            value = _format_scalar(value, build)
        elif type(value) == _list:
            value = [_format_scalar(x, build) for x in value]
        out[key] = value
    return out
