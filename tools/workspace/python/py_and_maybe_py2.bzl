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

_str = type("")
_list = type([])

def _format(value, fmt):
    if type(value) == _str:
        return value.format(**fmt)
    elif type(value) == _list:
        return [x.format(**fmt) for x in value]
    else:
        fail("Unsupported type: {}".format(type(value)))

def py_and_maybe_py2(
        rule,
        kwargs_config,
        kwargs):
    version_major, _ = PYTHON_VERSION.split(".")
    version2_major, _ = PYTHON2_VERSION.split(".")
    if version2_major != "2":
        fail("Unknown error?")
    fmt_py = dict(
        version_major = version_major,
        maybe_version_major = "",
        version = PYTHON_VERSION,
        site_packages = PYTHON_SITE_PACKAGES_RELPATH,
    )
    fmt_py2 = dict(
        version_major = version2_major,
        maybe_version_major = version2_major,
        version = PYTHON2_VERSION,
        site_packages = PYTHON2_SITE_PACKAGES_RELPATH,
    )
    # print("\npy:\n{}\n\n\npy2:\n{}\n\n".format(fmt_py, fmt_py2))
    kwargs_py = dict()  # Bazel version.
    kwargs_py2 = dict()  # Specific version.
    for key, value in kwargs_config.items():
        kwargs_py[key] = _format(value, fmt_py)
        kwargs_py2[key] = _format(value, fmt_py2)
    kwargs_py += kwargs
    kwargs_py2 += kwargs
    # print("\npy:\n{}\n\n\npy2:\n{}\n\n".format(kwargs_py, kwargs_py2))
    rule(**kwargs_py)
    if version_major != version2_major:
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
