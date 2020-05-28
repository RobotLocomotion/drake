# -*- python -*-

load("@drake//tools/install:install.bzl", "InstallInfo")

# List of exact file names of license files
LICENSE_LITERALS = [
    "BSD-LICENSE",  # ccd
    "COPYING",
    "Copyright.txt",  # vtk
    "EULA.pdf",  # gurobi
    "LICENSE",
    "mosek-eula.pdf",  # mosek
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
    # Don't check empty installs (can happen if an install is a dummy due to
    # some platforms relying on a package already being installed).
    if not label[InstallInfo].install_actions:
        return []

    # Look for file(s) that appear to be license(s) in the install actions.
    has_license = False
    for a in label[InstallInfo].install_actions:
        if hasattr(a, "src") and _is_license_file(a.src.basename):
            has_license = True

    # If no license found, return the failing label.
    if not has_license:
        return [label.label]

    # Otherwise return nothing; caller collects failing labels to report.
    return []

#------------------------------------------------------------------------------
def _check_licenses_impl(ctx):
    # Iterate over labels, collecting ones that are missing licenses.
    labels_missing_licenses = []
    for label in ctx.attr.install_labels:
        labels_missing_licenses += _check_licenses_for_label(label)

    # Report collected failures.
    if labels_missing_licenses:
        fail("Missing license files for install label(s) %s" %
             labels_missing_licenses)

_check_licenses = rule(
    attrs = {
        "install_labels": attr.label_list(providers = [InstallInfo]),
    },
    implementation = _check_licenses_impl,
)

def check_licenses(install_labels, name = "check_licenses"):
    """Check that install labels include license files.

    Given a list of install labels (e.g. ``//package:install``), check that the
    set of files installed by the label includes one or more files that appear
    to provide a license, by checking the file names against a list of known
    names of license files (e.g. ``LICENSE``).

    This is used to verify that a set of packages is installing the license
    files for those packages.

    Args:
        name (:obj:`str`): Rule name (default = ``"check_licenses"``).
        install_labels (:obj:`list` of :obj:`Label`): List of install labels
            (must be created by :func:`install` or :func:`install_files`) to
            be checked.
    """
    _check_licenses(
        name = name,
        install_labels = install_labels,
    )
