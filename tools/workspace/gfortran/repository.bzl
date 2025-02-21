load("//tools/workspace:execute.bzl", "execute_or_fail", "which")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def _gfortran_impl(repo_ctx):
    # Find the compiler.
    compiler = which(repo_ctx, "gfortran")
    if not compiler:
        fail("Could not find gfortran")
    compiler = str(compiler)
    repo_ctx.symlink(compiler, "gfortran-found")

    # Transcribe which version we found.
    dumpversion = execute_or_fail(repo_ctx, [compiler, "-dumpversion"]).stdout
    repo_ctx.file(
        "version.bzl",
        content = "MAJOR = {}\n".format(int(dumpversion.strip())),
    )

    # Transcribe the library link path.
    filename = execute_or_fail(
        repo_ctx,
        [
            compiler,
            "--print-file-name=libgfortran.{}".format(
                "dylib" if repo_ctx.os.name == "mac os x" else "so",
            ),
        ],
    ).stdout
    libdir = repo_ctx.path(filename.strip()).dirname
    repo_ctx.file(
        "path.bzl",
        content = "LIBDIR = {}\n".format(repr(str(libdir))),
    )

    # Add the build file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/gfortran:package.BUILD.bazel"),
        "BUILD.bazel",
    )

_gfortran_repository = repository_rule(
    doc = """
        Locate gfortran and alias it to `:compiler`; locate libgfortran
        (and libquadmath when relevant) and alias them to `:runtime`.
    """,
    local = True,
    configure = True,
    implementation = _gfortran_impl,
)

def gfortran_repository(
        name,
        _is_drake_self_call = False,
        **kwargs):
    if not _is_drake_self_call:
        print_warning("gfortran_repository")
    _gfortran_repository(
        name = name,
        **kwargs
    )
