load(
    "@python//:version.bzl",
    "PYTHON_VERSION",
    "PYTHON_SITE_PACKAGES_RELPATH",
)
load(
    "@python2//:version.bzl",
    PYTHON2_VERSION="PYTHON_VERSION",
    PYTHON2_SITE_PACKAGES_RELPATH="PYTHON_SITE_PACKAGES_RELPATH",
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
    Provides Bazel Python (2 or 3) and Python2 targets, without duplication /
    conflict. This should be used **sparingly**, only when Python2 and Python3
    support is simultaneously needed.

    @param rule
        Macro describing rule (e.g. `py_library`, `cc_binary`).
    @param kwargs_py
        Arguments for Bazel Python target.
    @param kwargs_py2
        Arguments for Python2 target.

    For Bazel Python versions:
    - 2: Defines a Python2 target with `kwargs + kwargs_py`, and creates an
    alias for the Python2 name (if its unique).
    - 3: Defines a Python3 target with `kwargs + kwargs_py` and Python2 target
    with `kwargs_py3`.

    Formatting:
        Values can be strings or lists of strings; any other types are ignored.
        The following format variables are available:
            {py_major}: Replaces with Python major version.
            {py_site_packages}: Relpath for site-packages installation.
    """
    if _PY2["py_major"] != "2":
        fail("@python2 is misconfigured: {}".format(_PY2))
    kwargs_py = _format(kwargs + kwargs_py, _PY)  # Bazel Python
    kwargs_py2 = _format(kwargs + kwargs_py2, _PY2)  # Python2
    rule(**kwargs_py)
    if _PY["py_major"] != _PY2["py_major"]:
        # Duplicate the rule.
        if kwargs_py2["name"] == kwargs_py["name"]:
            fail(("Python2 name '{}' should not match Bazel Python " + 
                  "name '{}'").format(kwargs_py2["name"], kwargs_py["name"]))
        rule(**kwargs_py2)
    else:
        # Alias the rule if the names do not conflict.
        if kwargs_py2["name"] != kwargs_py["name"]:
            native.alias(
                name = kwargs_py2["name"],
                actual = kwargs_py["name"],
                visibility = kwargs.get("visibility"),
            )
