# -*- mode: python -*-

# These macros are intended to be used when declaring tests that either may-use
# or must-use either Gurobi or Mosek commercial solvers.  These labels both
# account for any license-server specific needs (such as network access or rate
# control), as well as provide a marker so that //tools/bazel.rc can
# selectively enable tests based on the developer's chosen configuration.

def gurobi_test_tags(gurobi_required = True):
    """Returns the test tags necessary for properly running Gurobi tests.

    By default, sets gurobi_required=True, which will require that the supplied
    tag filters include "gurobi".

    Gurobi checks a license file, and may need to contact a license server to
    check out a license. Therefore, tests that use Gurobi must have the tag
    "local", because they are non-hermetic. For the moment, we also require
    the tag "exclusive", to rate-limit license servers with a small number of
    licenses.
    """
    # TODO(david-german-tri): Find a better fix for the license server problem.
    nominal_tags = [
        "exclusive",
        "local",
    ]
    if gurobi_required:
        return nominal_tags + ["gurobi"]
    else:
        return nominal_tags

def mosek_test_tags(mosek_required = True):
    """Returns the test tags necessary for properly running mosek tests.

    By default, sets mosek_required=True, which will require that the supplied
    tag filters include "mosek".

    MOSEK checks a license file, and may need to contact a license server to
    check out a license. Therefore, tests that use MOSEK must have the tag
    "local", because they are non-hermetic.
    """
    nominal_tags = [
        "local",
    ]
    if mosek_required:
        return nominal_tags + ["mosek"]
    else:
        return nominal_tags
