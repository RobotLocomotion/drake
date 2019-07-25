# -*- mode: python -*-

# These macros are intended to be used when declaring tests that either may-use
# or must-use dependencies that have constraints (commercial licenses, or
# peculiar behavior). For commercial dependnecies, these labels both account
# for any license-related needs and provide a marker so that //tools/bazel.rc
# can selectively enable tests based on the developer's chosen configuration.

def gurobi_test_tags(gurobi_required = True):
    """Returns the test tags necessary for properly running Gurobi tests.

    By default, sets gurobi_required=True, which will require that the supplied
    tag filters include "gurobi".

    Gurobi checks a license file outside the workspace so tests that use Gurobi
    must have the tag "no-sandbox". For the moment, we also require the tag
    "exclusive" to rate-limit license servers with a small number of licenses.
    """

    # TODO(david-german-tri): Find a better fix for the license server problem.
    nominal_tags = [
        "exclusive",  # implies "local"
        "no-sandbox",
    ]
    if gurobi_required:
        return nominal_tags + ["gurobi"]
    else:
        return nominal_tags

def mosek_test_tags(mosek_required = True):
    """Returns the test tags necessary for properly running MOSEK tests.

    By default, sets mosek_required=True, which will require that the supplied
    tag filters include "mosek".

    MOSEK checks a license file outside the workspace, so tests that use MOSEK
    must have the tag "no-sandbox".
    """
    nominal_tags = [
        "no-sandbox",
    ]
    if mosek_required:
        return nominal_tags + ["mosek"]
    else:
        return nominal_tags

def vtk_test_tags():
    """Returns test tags necessary for properly running VTK rendering tests
    locally.
    """
    return [
        # Defects related to platform-specific rendering-related libraries
        # when run under DRD or Helgrind.
        "no_drd",
        "no_helgrind",
        # Disable under LeakSanitizer and Valgrind Memcheck due to
        # driver-related leaks. For more information, see #7520.
        "no_lsan",
        "no_memcheck",
        # Mitigates driver-related issues when running under `bazel test`. For
        # more information, see #7004.
        "no-sandbox",
    ]
