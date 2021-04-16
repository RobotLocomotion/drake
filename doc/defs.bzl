# -*- python -*-

# This file contains build macros and constants that help define Drake's
# documentation targets (i.e., these should only be used for BUILD files
# within @drake//doc/...).

# Unless `setup/ubuntu/install_prereqs.sh --with-doc-only` has been run, most
# tests in //doc/... will fail to pass, so by default we'll disable them.
#
# A developer will have to explicitly opt-in in order to test these.
DEFAULT_TEST_TAGS = [
    "manual",
    # None of our documentation tools should hit the internet, but their
    # ecosystems might be doing so without us being aware.
    "block-network",
]

def _enumerate_filegroup_impl(ctx):
    out = ctx.actions.declare_file(ctx.attr.name)
    runpaths = {}
    for x in ctx.attr.data:
        for y in x.data_runfiles.files.to_list():
            if y.short_path.startswith("../"):
                runpath = y.short_path[3:]
            else:
                runpath = ctx.workspace_name + "/" + y.short_path
            runpaths[runpath] = True
    result = sorted(runpaths.keys())
    ctx.actions.write(out, "\n".join(result) + "\n")

    # Return the new file to our caller.
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]

enumerate_filegroup = rule(
    implementation = _enumerate_filegroup_impl,
    doc = """
Creates a text file listing the files incorporated into the given filegroup.

The listing is transitive (includes both filegroup.srcs and filegroup.data).
https://docs.bazel.build/versions/master/be/general.html#filegroup
""",
    attrs = {
        "data": attr.label_list(
            allow_empty = False,
            doc = "Filegroup whose data we should enumerate",
        ),
    },
)
