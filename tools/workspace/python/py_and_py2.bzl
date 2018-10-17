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

_PY = dict(
    py_major = PYTHON_VERSION.split(".")[0],
    py_site_packages = PYTHON_SITE_PACKAGES_RELPATH,
)
_PY2 = dict(
    py_major = PYTHON2_VERSION.split(".")[0],
    py_site_packages = PYTHON2_SITE_PACKAGES_RELPATH,
)

def _format(raw, fmt):
    out = dict()
    for key, value in raw.items():
        # N.B. `str` and `list` do not work in this comparison.
        if type(value) == type(""):
            value = value.format(**fmt)
        elif type(value) == type([]):
            value = [x.format(**fmt) for x in value]
        out[key] = value
    return out

def py_and_py2(
        rule,
        kwargs_py,
        kwargs_py2,
        **kwargs):
    """
    Provides Bazel Python (2 or 3) and Python2 targets, without duplication or
    conflict. This should be used **sparingly**, only when Python2 and Python3
    support is needed within the same build.

    If Bazel Python is Python2:
        Defines a Python2 target with `kwargs + kwargs_py`, and creates an
    alias for the Python2 name (if it's unique).
    If Bazel Python is Python3:
        Defines a Python3 target with `kwargs + kwargs_py` and Python2 target
    with `kwargs_py3`.

    @param rule
        Starlark rule (e.g. `py_library`, `cc_binary`).
    @param kwargs_py
        Arguments for Bazel Python target.
    @param kwargs_py2
        Arguments for Python2 target.

    Strings and lists of strings in `kwargs`, `kwargs_py`, `kwargs_py2` can be
    formatted by the following variables:
        {py_major}: Python major version.
        {py_site_packages}: Relpath for site-packages installation.
    """
    if _PY2["py_major"] != "2":
        fail("@python2 has the wrong major version: {}".format(_PY2))

    # Define Bazel Python target.
    kwargs_py = _format(kwargs + kwargs_py, _PY)
    rule(**kwargs_py)

    # Define Python2...
    kwargs_py2 = _format(kwargs + kwargs_py2, _PY2)
    if _PY["py_major"] == "2":
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
