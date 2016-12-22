# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# GUROBI_PATH should be the linux64 directory in the Gurobi 6.05 release.
# TODO(david-german-tri): Add support for OS X.
def _gurobi_impl(repository_ctx):
    gurobi_path = repository_ctx.os.environ.get('GUROBI_PATH')
    if gurobi_path:
        repository_ctx.symlink(gurobi_path, "gurobi-distro")
        repository_ctx.symlink(getattr(repository_ctx.attr, "workspace_dir") +
                               "/" + getattr(repository_ctx.attr, "build_file"),
                               "BUILD")

gurobi_repository = repository_rule(
    attrs = {
        "build_file": attr.string(mandatory = True),
        "workspace_dir": attr.string(mandatory = True),
    },
    local = True,
    implementation = _gurobi_impl,
)
