# -*- python -*-

def _impl(repo_ctx):
    name = repo_ctx.attr.name
    date = repo_ctx.attr.date
    cc_aliases = repo_ctx.attr.cc_aliases

    build = "package(default_visibility = [\"//visibility:public\"])\n"
    deprecation = "".join([
        "DRAKE DEPRECATED: The @{} external is deprecated".format(name),
        " and will be removed from Drake on or after {}.".format(date),
    ])
    for label, actual in cc_aliases.items():
        build += "cc_library({})\n".format(", ".join([
            "name = " + repr(label),
            "deps = [" + repr(actual) + "]",
            "deprecation = " + repr(deprecation),
        ]))

    repo_ctx.file("BUILD.bazel", build)

add_deprecation = repository_rule(
    doc = """Adds a repository rule with deprecated aliases to other targets.
    This is particularly useful when renaming an external repository.

    Example:
        add_deprecation(
            name = "qhull",
            date = "2038-01-19",
            cc_aliases = {"qhull": "@qhull_internal//:qhull"},
        )
    """,
    attrs = {
        "date": attr.string(
            doc = "Scheduled removal date of the deprecated target(s).",
            mandatory = True,
        ),
        "cc_aliases": attr.string_dict(
            doc = """
            Optional mapping for cc_library deprecations. The keys are
            deprecated target names, the values are the non-deprecated labels.
            """,
        ),
    },
    implementation = _impl,
)
