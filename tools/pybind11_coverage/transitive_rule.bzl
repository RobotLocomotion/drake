# -*- python -*-

# For getting pybind coverage statistics, we want to get the source files
# inside the `bindings/pydrake` directory.  We can't just glob them directly
# because of the presence of subpackages inside bindings/pydrake package, so we
# need to use providers to get them.  We add a rule in each `package` for which
# we want the `*.cc` using the macro `add_for_pybind_coverage`.

GlobFiles = provider("files")

def _file_globber_impl(ctx):
    all_files = []
    for f in ctx.attr.files:
        all_files = all_files + f.files.to_list()
    all_files_depset = depset(all_files)

    return [
        GlobFiles(files = all_files_depset),
        DefaultInfo(files = all_files_depset),
    ]

files_globber = rule(
    implementation = _file_globber_impl,
    attrs = {
        "files": attr.label_list(allow_files = True),
    },
)

def get_trans_deps(deps):
    depset_list = ([dep[GlobFiles].files for dep in deps])
    files_list = []
    for dep in depset_list:
        for f in dep.to_list():
            files_list.append(f)
    return depset(files_list)

def _generate_pybind_coverage_impl(ctx):
    script = ctx.executable.script
    trans_srcs = get_trans_deps(ctx.attr.deps)
    srcs_list = trans_srcs.to_list()

    # The positional arguments is the list of source files for the Python
    # bindings.
    xml_file = ctx.attr.xml_docstrings.files.to_list()[0]
    all_args = [src.path for src in srcs_list] + [
        "--file_coverage",
        ctx.outputs.file_coverage.path,
        "--class_coverage",
        ctx.outputs.class_coverage.path,
        "--xml_docstrings",
        xml_file.path,
    ]

    ctx.actions.run(
        executable = script,
        arguments = all_args,
        inputs = srcs_list + [xml_file],
        tools = [script],
        outputs = [ctx.outputs.file_coverage, ctx.outputs.class_coverage],
    )

def add_for_pybind_coverage():
    files_globber(
        name = "cc_glob",
        files = native.glob(["*.cc"]),
        visibility = ["//bindings/pydrake:__pkg__"],
    )

generate_pybind_coverage = rule(
    implementation = _generate_pybind_coverage_impl,
    attrs = {
        "deps": attr.label_list(allow_files = True),
        "script": attr.label(
            allow_files = True,
            executable = True,
            cfg = "host",
        ),
        "file_coverage": attr.output(mandatory = True),
        "class_coverage": attr.output(mandatory = True),
        "xml_docstrings": attr.label(allow_single_file = True),
    },
)
