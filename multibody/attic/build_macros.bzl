# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_test",
)

def attic_drake_cc_binary(**kwargs):
    """A wrapper to that should be exclusively used within multibody/attic."""
    drake_cc_binary(
        **kwargs
    )

def attic_drake_cc_googletest(**kwargs):
    """A wrapper to that should be exclusively used within multibody/attic."""
    drake_cc_googletest(**kwargs)

def attic_drake_cc_library(**kwargs):
    """A wrapper to that should be exclusively used within multibody/attic."""
    drake_cc_library(
        strip_include_prefix = "/multibody/attic",
        include_prefix = "drake/multibody",
        **kwargs
    )

def attic_drake_cc_test(**kwargs):
    """A wrapper to that should be exclusively used within multibody/attic."""
    drake_cc_test(**kwargs)

# These are lists of labels that we should alias from //mulibody/foo to resolve
# to //multibody/alias/foo -- deprecation shims so that Bazel downstreams are
# not affected by the attic move.
#
# These lists were created via:
#   bazel query 'visible(@stx//:stx,
#       //multibody/attic/... except tests(//multibody/attic/...))' |
#     fgrep -v 'installed_headers' | sort
#
# In other words -- find all of the public targets within the attic.

# TODO(jwnimmer-tri) Add more lists as we move more folders into the attic.

# Labels in //multibody/attic/parsers:*.
_PARSERS_LABELS = [
    "find_files",
    "parsers",
    "test_models",
]

def _add_attic_aliases(short_labels, subdir):
    # Declare aliases for `short_labels` within `subdir`.
    for item in short_labels:
        # Alias the target itself.
        full_item = "//multibody/attic" + subdir + ":" + item
        native.alias(
            name = item,
            actual = full_item,
        )

        # For some targets, we have to alias the installed_headers also.
        if "models" in item:
            continue  # Data, not cc_library.
        if "genproto" in item:
            continue  # Genrule, not cc_library.
        native.alias(
            name = item + ".installed_headers",
            actual = full_item + ".installed_headers",
        )

# Each one of the below macros should each be called (one) from the matching
# package they refer to.

# TODO(jwnimmer-tri) Add more macros as we move more folders into the attic.

def add_attic_aliases_parsers():
    _add_attic_aliases(_PARSERS_LABELS, "/parsers")
