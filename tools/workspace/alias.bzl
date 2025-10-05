def _impl(repo_ctx):
    name = repo_ctx.attr.name
    aliases = repo_ctx.attr.aliases

    build_files = {}
    for target, actual in aliases.items():
        if target.startswith("//"):
            subdir, name = target[2:].split(":")
            path = "{}/BUILD.bazel".format(subdir)
        else:
            name = target
            path = "BUILD.bazel"
        if path in build_files:
            text = build_files[path]
        else:
            text = "package(default_visibility = [\"//visibility:public\"])\n"
        text += "alias({})\n".format(", ".join([
            "name = " + repr(name),
            "actual = " + repr(actual),
        ]))
        build_files[path] = text

    for path, text in build_files.items():
        repo_ctx.file(path, text)

alias_repository = repository_rule(
    doc = """Adds a repository with aliases to other repositories.""",
    attrs = {
        "aliases": attr.string_dict(
            doc = """
            Dictionary of aliases to create. The keys are target names, the
            values are the destination labels (i.e., the 'actual').

            Target names without a leading `//` are placed into the top-level
            package. To add aliases in other packages, provide the alias as a
            complete label (e.g., `//a/b:c` will generate `a/b/BUILD.bazel`
            with `alias(name = 'c', actual = ...)`).
            """,
        ),
    },
    implementation = _impl,
)
