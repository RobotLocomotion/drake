def _impl(repo_ctx):
    # Symlink the relevant headers.
    hdrs = [
        "dmumps_c.h",
        "mumps_c_types.h",
        "mumps_compat.h",
        "mumps_int_def.h",
        "mumps_seq/elapse.h",
        "mumps_seq/mpi.h",
        "mumps_seq/mpif.h",
    ]
    for hdr in hdrs:
        repo_ctx.symlink("/usr/include/" + hdr, "include/" + hdr)

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/mumps_internal:package.BUILD.bazel"),
        "BUILD.bazel",
    )

mumps_internal_repository = repository_rule(
    doc = """Adds a repository rule for the host mumps library from Ubuntu.
    This repository is not used on macOS.
    """,
    local = True,
    implementation = _impl,
)
