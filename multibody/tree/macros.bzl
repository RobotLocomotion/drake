load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_library",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

_TEMPLATE = """
#pragma once

#include "drake/multibody/tree/internal/{header}"

#warning "This header is deprecated, and will be removed around 2019/03/01."
""".lstrip()

def drake_cc_forwarding_hdrs_tree_internal(
        name,
        hdrs = None,
        visibility = None):
    """Specific version of `drake_cc_forwarding_hdrs` in that it does not
    suggest what the original headers are, and mentions a deprecation date."""
    hdrs or fail("`hdrs` must be supplied")
    for header in hdrs:
        generate_file(
            name = header,
            content = _TEMPLATE.format(header = header),
            visibility = visibility,
        )
    drake_cc_library(
        name = name,
        hdrs = hdrs,
        visibility = visibility,
    )
