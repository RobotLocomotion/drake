def _impl(repo_ctx):
    # We are enabled only on linux, not macOS.
    enabled = repo_ctx.os.name == "linux"
    repo_ctx.file("defs.bzl", content = "ENABLED = {}\n".format(enabled))
    if not enabled:
        repo_ctx.file("error.txt", content = """
ERROR: The mumps_internal repository rule is disabled on this operating system.
""")

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
        Label("{}:package-{}.BUILD.bazel".format(
            "@drake//tools/workspace/mumps_internal",
            "enabled" if enabled else "error",
        )),
        "BUILD.bazel",
    )

mumps_internal_repository = repository_rule(
    doc = """Adds a repository rule for the host mumps library from Ubuntu.
    This repository is not useful on macOS; the repository rule will evaluate
    without error, but the cc_library would error if used in a build.
    """,
    local = True,
    implementation = _impl,
)
