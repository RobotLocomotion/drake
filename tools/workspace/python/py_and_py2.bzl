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

# `str` and `list` do not seem to make the correct comparison below.
_str = type("")
_list = type([])

def _format(value, fmt):
    if type(value) == _str:
        return value.format(**fmt)
    elif type(value) == _list:
        return [x.format(**fmt) for x in value]
    else:
        fail("Unsupported type: {}".format(type(value)))

def py_and_py2(
        rule,
        kwargs_config,
        kwargs):
    """
    Guarantees that there will be an explicit Bazel Python and Python2 rule for
    a target. This should be used **sparingly**, only when both the Bazel
    Python and Python2 support is explicitly needed.

    The Bazel Python version (py) can be 2 or 3 depending on user
    configuration, while we always want access to Python2 versions (py2), thus
    we can have the following cases:

    Bazel Python 2: We have a target, and an alias (if necessary) for the
        Python2 name
    Bazel Python 3: We have a target for Python3 and a Python2 versions.

    @param rule
        Macro describing rule (e.g. `py_library`, `cc_binary`).
    @param kwargs_config
        Keyword arguments that should be formatted.
        Values can be strings or lists of strings, with the following format
        variables:
            {py_2_or_3}: Replaces with '2' or '3' depending on Python version,
                regardless of Bazel Python or Python2.
            {py_blank_or_3}: Replaces with '' for Bazel Python and '2'
                for Python2.
            {py_site_packages}: Relpath for site-packages installation.
    @param kwargs
        Keyword arguments that need no formatting.
    """
    py_major, _ = PYTHON_VERSION.split(".")
    py2_major, _ = PYTHON2_VERSION.split(".")
    if py2_major != "2":
        fail("Unknown error?")
    fmt_py = dict(
        py_2_or_3 = py_major,
        py_blank_or_2 = "",
        py_site_packages = PYTHON_SITE_PACKAGES_RELPATH,
    )
    fmt_py2 = dict(
        py_2_or_3 = py2_major,
        py_blank_or_2 = py2_major,
        version = PYTHON2_VERSION,
        py_site_packages = PYTHON2_SITE_PACKAGES_RELPATH,
    )
    kwargs_py = dict()  # Bazel version.
    kwargs_py2 = dict()  # Specific version.
    for key, value in kwargs_config.items():
        kwargs_py[key] = _format(value, fmt_py)
        kwargs_py2[key] = _format(value, fmt_py2)
    kwargs_py += kwargs
    kwargs_py2 += kwargs
    rule(**kwargs_py)
    if py_major != py2_major:
        # Duplicate the rule.
        if kwargs_py2["name"] == kwargs_py["name"]:
            fail("Python2 name '{}' should not match Python name '{}'".format(
                kwargs_py2["name"], kwargs_py["name"]))
        rule(**kwargs_py2)
    else:
        # Alias the rule if the names do not conflict.
        if kwargs_py2["name"] != kwargs_py["name"]:
            native.alias(
                name = kwargs_py2["name"],
                actual = kwargs_py["name"],
                visibility = kwargs.get("visibility"),
            )
