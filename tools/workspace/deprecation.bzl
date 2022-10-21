# -*- python -*-

def _split_label(label):
    """Returns the (package, name) for a given string label.
    Does not support repository names nor //.  For example:
      "quux" => ("", "quux")
      "foo/bar:quux" => ("foo/bar", "quux")
    """
    if ":" in label:
        return label.rsplit(":", 1)
    if "/" in label:
        fail("Malformed label " + label)
    return ("", label)

def _impl(repo_ctx):
    name = repo_ctx.attr.name
    date = repo_ctx.attr.date
    cc_aliases = repo_ctx.attr.cc_aliases
    py_aliases = repo_ctx.attr.py_aliases
    aliases = repo_ctx.attr.aliases

    deprecation = "".join([
        "DRAKE DEPRECATED: The @{} external is deprecated".format(name),
        " and will be removed from Drake on or after {}.".format(date),
    ])

    # A map from package name to BUILD.bazel contents for that package.
    build_content = {"": ""}
    for label, actual in cc_aliases.items():
        package, name = _split_label(label)
        content = build_content.get(package, "")
        content += "cc_library({})\n".format(", ".join([
            "name = " + repr(name),
            "deps = [" + repr(actual) + "]",
            "deprecation = " + repr(deprecation),
        ]))
        build_content[package] = content
    for label, actual in py_aliases.items():
        package, name = _split_label(label)
        content = build_content.get(package, "")
        build += "py_library({})\n".format(", ".join([
            "name = " + repr(name),
            "deps = [" + repr(actual) + "]",
            "deprecation = " + repr(deprecation),
        ]))
        build_content[package] = content
    for label, actual in aliases.items():
        package, name = _split_label(label)
        content = build_content.get(package, "")
        content += "alias({})\n".format(", ".join([
            "name = " + repr(name),
            "actual = " + repr(actual),
            # Unfortunately, Bazel does not obey `deprecation =` on an alias().
        ]))
        build_content[package] = content
    if aliases or (not cc_aliases and not py_aliases):
        # If there are any targets without a deprecation attribute, or if there
        # are no targets in the first place, then we must deprecated the entire
        # BUILD file.
        package = ""
        content = build_content.get(package, "")
        content += "print(" + repr(deprecation) + ")\n"
        build_content[package] = content

    prologue = "package(default_visibility = [\"//visibility:public\"])\n"
    for package, content in build_content.items():
        content = prologue + content
        if package:
            repo_ctx.file("{}/BUILD.bazel".format(package), content)
        else:
            repo_ctx.file("BUILD.bazel", content)

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
