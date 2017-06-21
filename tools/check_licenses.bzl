# -*- python -*-

# List of exact file names of license files
LICENSE_LITERALS = [
    "BSD-LICENSE",  # ccd
    "COPYING",
    "Copyright.txt",  # vtk
    "LICENSE",
]

# List of file name prefixes of license files
LICENSE_PREFIXES = [
    "COPYING.",
    "LICENSE.",
]

#------------------------------------------------------------------------------
def _is_license_file(filename):
    # Check literal file names
    if filename in LICENSE_LITERALS:
        return True

    # Check file prefixes
    for p in LICENSE_PREFIXES:
        if filename.startswith(p):
            return True

    # No match
    return False

#------------------------------------------------------------------------------
def _check_licenses_for_label(label):
    has_license = False
    for a in label.install_actions:
        if _is_license_file(a.src.basename):
            has_license = True

    if not has_license:
        fail("Install label %s is missing license files" % label.label)

#------------------------------------------------------------------------------
def _check_licenses_impl(ctx):
    for l in ctx.attr.install_labels:
        _check_licenses_for_label(l)

_check_licenses = rule(
    attrs = {
        "install_labels": attr.label_list(providers = ["install_actions"]),
    },
    implementation = _check_licenses_impl,
)

def check_licenses(install_labels, name = "check_licenses"):
    _check_licenses(
        name = name,
        install_labels = install_labels,
    )
