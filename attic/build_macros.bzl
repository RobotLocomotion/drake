# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_package_library",
    "drake_cc_test",
)

def attic_drake_cc_binary(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_binary(
        **kwargs
    )

def attic_drake_cc_googletest(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_googletest(**kwargs)

def attic_drake_cc_library(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_library(
        strip_include_prefix = "/attic",
        **kwargs
    )

def attic_drake_cc_package_library(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_package_library(
        **kwargs
    )

def attic_drake_cc_test(**kwargs):
    """A wrapper to that should be exclusively used within attic/...."""
    drake_cc_test(**kwargs)

def add_attic_aliases(short_labels):
    """For each item it short_labels, declares //current_package:item as an
    alias to actual label //attic/current_package:item.

    The aliases serve as deprecation shims so that Bazel downstreams are not
    affected by files moving into the attic.

    To obtain a comprehensive list of what to pass into `short_labels`, you
    may use something similar to this query:

      bazel query 'visible(@stx//:stx,
          //attic/somepkg/... except tests(//attic/somepkg/...))' |
          fgrep -v 'installed_headers' | sort

    (In other words -- find all of the public targets within the attic.)
    """

    # The name (e.g., "multibody/parsers") of the package we're aliasing.
    subdir = native.package_name()

    # Declare aliases for `short_labels` within `subdir`.
    for item in short_labels:
        # Alias the target itself.
        full_item = "//attic/" + subdir + ":" + item
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
