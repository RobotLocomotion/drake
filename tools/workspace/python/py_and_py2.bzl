load(
    "@python//:version.bzl",
    "PYTHON_EXT_SUFFIX",
    "PYTHON_SITE_PACKAGES_RELPATH",
    "PYTHON_VERSION",
)
load(
    "@python2//:version.bzl",
    PYTHON2_EXT_SUFFIX = "PYTHON_EXT_SUFFIX",
    PYTHON2_SITE_PACKAGES_RELPATH = "PYTHON_SITE_PACKAGES_RELPATH",
    PYTHON2_VERSION = "PYTHON_VERSION",
)

_BAZEL = struct(
    version_field = "py" + PYTHON_VERSION.split(".")[0],
    ext_suffix = PYTHON_EXT_SUFFIX,
    major = PYTHON_VERSION.split(".")[0],
    site_packages = PYTHON_SITE_PACKAGES_RELPATH,
)
_PY2 = struct(
    version_field = "py2",
    ext_suffix = PYTHON2_EXT_SUFFIX,
    major = PYTHON2_VERSION.split(".")[0],
    site_packages = PYTHON2_SITE_PACKAGES_RELPATH,
)

# Cannot use `type(x) == str`, etc., but can infer from literals?
_list = type([])
_str = type("")
_struct = type(struct())

def py_select(py2, py3):
    return struct(py2 = py2, py3 = py3)

def py2_py3(
        rule,
        alias = None,
        **kwargs):
    """
    Provides Bazel Python and Python2 targets, without duplication or conflict.

    If Bazel is using Python2, only a Python2 target will be defined.
    If Bazel using Python3, both a Python2 and Python3 target will be defined.

    @param rule
        Starlark rule (e.g. `py_library`, `cc_binary`).
    @param alias
        If specified, will create an alias to the version of the target for
        Bazel.
    @param kwargs
        Arguments for the rule. that are resolve in the following fashion:
        * If an argument's value comes from `py_select`, it will be replaced
        based on the Python version using the `py2` or `py3` values.
        * Strings have the following tokens replaced:
            @PYTHON_SITE_PACKAGES@
            @PYTHON_EXT_SUFFIX@
    """

    # Define Python2 target.
    if _PY2.major != "2":
        fail("@python2 has the wrong major version: {}".format(_PY2))
    kwargs_py2 = _format(kwargs, _PY2)
    rule(**kwargs_py2)
    alias_actual = kwargs_py2["name"]

    if _BAZEL.major == "3":
        # Define Python3 target.
        kwargs_py3 = _format(kwargs, _BAZEL)
        alias_actual = kwargs_py3["name"]
        rule(**kwargs_py3)

    # Define alias.
    if alias:
        native.alias(
            name = alias,
            actual = alias_actual,
            visibility = kwargs_py2.get("visibility"),
        )

def _format_raw(value, build):
    subs = (
        ("@PYTHON_SITE_PACKAGES@", build.site_packages),
        ("@PYTHON_EXT_SUFFIX@", build.ext_suffix),
    )
    for old, new in subs:
        value = value.replace(old, new)
    return value

def _format_item(value, build):
    # N.B. `str` and `list` do not work in this comparison.
    if type(value) == _str:
        return _format_raw(value, build)
    elif type(value) == _list:
        return [_format_raw(x, build) for x in value]
    else:
        return value

def _format(raw, build):
    out = dict()
    for key, value in raw.items():
        if type(value) == _struct:
            value = getattr(value, build.version_field)
        out[key] = _format_item(value, build)
    return out
