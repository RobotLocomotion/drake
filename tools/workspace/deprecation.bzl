def _impl(repo_ctx):
    name = repo_ctx.attr.name
    date = repo_ctx.attr.date
    cc_aliases = repo_ctx.attr.cc_aliases
    py_aliases = repo_ctx.attr.py_aliases
    aliases = repo_ctx.attr.aliases

    build = "load(\"@drake//tools/skylark:cc.bzl\", \"cc_library\")\n"
    build += "load(\"@drake//tools/skylark:py.bzl\", \"py_library\")\n"
    build += "package(default_visibility = [\"//visibility:public\"])\n"
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
    for label, actual in py_aliases.items():
        build += "py_library({})\n".format(", ".join([
            "name = " + repr(label),
            "deps = [" + repr(actual) + "]",
            "deprecation = " + repr(deprecation),
        ]))
    for label, actual in aliases.items():
        # Unfortunately, Bazel does not obey `deprecation = ...` on an alias().
        build += "alias({})\n".format(", ".join([
            "name = " + repr(label),
            "actual = " + repr(actual),
        ]))
    if aliases or (not cc_aliases and not py_aliases):
        # If there are any targets without a deprecation attribute, or if there
        # are no targets in the first place, then we must deprecated the entire
        # BUILD file.
        build += "print(" + repr(deprecation) + ")\n"

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
        "py_aliases": attr.string_dict(
            doc = """
            Optional mapping for py_library deprecations. The keys are
            deprecated target names, the values are the non-deprecated labels.
            """,
        ),
        "aliases": attr.string_dict(
            doc = """
            Optional mapping for any other deprecations. The keys are
            deprecated target names, the values are the non-deprecated labels.

            Note that (in contrast to the cc or py aliases) these labels
            do NOT generate deprecation warnings when they are used. Instead,
            the BUILD file will print a warning when it's loaded, even if none
            of its targets are used as dependencies.
            """,
        ),
    },
    implementation = _impl,
)
