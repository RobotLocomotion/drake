def _impl(repo_ctx):
    name = repo_ctx.attr.name
    aliases = repo_ctx.attr.aliases

    build = "package(default_visibility = [\"//visibility:public\"])\n"
    for name, actual in aliases.items():
        build += "alias({})\n".format(", ".join([
            "name = " + repr(name),
            "actual = " + repr(actual),
        ]))

    repo_ctx.file("BUILD.bazel", build)

alias_repository = repository_rule(
    doc = """Adds a repository with aliases to other repositories.""",
    attrs = {
        "aliases": attr.string_dict(
            doc = """
            Dictionary of aliases to create. The keys are target names,
            the values are the destination labels (i.e., the 'actual').
            """,
        ),
    },
    implementation = _impl,
)
