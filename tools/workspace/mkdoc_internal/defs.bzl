load("//tools/skylark:cc.bzl", "CcInfo")

def _collect_cc_header_info(targets, hdr_subdir):
    """Given a list of targets (cc_library) and a specific physical
    subdirectory (e.g., "drake/multibody/tree"), scans for and returns
    the information necessary to run mkdoc on the headers from that
    specific subdirectory (only).
    """
    compile_flags = []
    transitive_headers_depsets = []
    target_headers_depsets = []
    for target in targets:
        if CcInfo in target:
            compilation_context = target[CcInfo].compilation_context

            for define in compilation_context.defines.to_list():
                compile_flags.append("-D{}".format(define))
            for system_include in compilation_context.system_includes.to_list():
                system_include = system_include or "."
                compile_flags.append("-isystem{}".format(system_include))
            for include in compilation_context.includes.to_list():
                include = include or "."
                compile_flags.append("-I{}".format(include))
            for quote_include in compilation_context.quote_includes.to_list():
                quote_include = quote_include or "."
                compile_flags.append("-iquote{}".format(quote_include))

            # Find any headers that match the hdr_subdir filter. We want the
            # virtual include path, not the source path, since only the former
            # has a corresponding -I flag. The virtual include path looks like:
            # "bazel-out/.../_virtual_includes/{target_name}/drake/...".
            target_headers = []
            for header in compilation_context.headers.to_list():
                if header.owner.workspace_root != target.label.workspace_root:
                    continue
                if not header.path.startswith("bazel-out"):
                    continue
                if "/_virtual_includes/" not in header.path:
                    continue
                tail = header.path.split("/_virtual_includes/", 1)[1]
                header_path = tail.split("/", 1)[1]

                # The header_path is like "drake/foo/bar/quux.h". Scrape it for
                # docstrings iff hdr_subdir is "drake/foo/bar".
                actual_subdir = header_path.rsplit("/", 1)[0]
                if actual_subdir == hdr_subdir:
                    target_headers.append(header)

            target_headers_depsets.append(depset(direct = target_headers))
            transitive_headers_depsets.append(compilation_context.headers)

    return struct(
        compile_flags = compile_flags,
        target_headers = depset(transitive = target_headers_depsets),
        transitive_headers = depset(transitive = transitive_headers_depsets),
    )

def _generate_pybind_documentation_header_impl(ctx):
    header_info = _collect_cc_header_info(
        ctx.attr.targets,
        ctx.attr.hdr_subdir,
    )
    if len(header_info.target_headers.to_list()) == 0:
        fail("No headers for " + ctx.outputs.out.path)

    args = ctx.actions.args()

    # TODO(jamiesnape): Remove this line when #14034 is resolved.
    args.add("-DDRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER")
    args.add("-DDRAKE_SPATIAL_ALGEBRA_HEADER")
    args.add_all(header_info.compile_flags, uniquify = True)
    outputs = [ctx.outputs.out]
    args.add("-output=" + ctx.outputs.out.path)
    args.add("-quiet")
    args.add("-root-name=" + ctx.attr.root_name)
    for p in ctx.attr.exclude_hdr_patterns:
        args.add("-exclude-hdr-patterns=" + p)
    args.add("-std=c++20")
    args.add("-w")
    args.add_all(header_info.target_headers)

    ctx.actions.run(
        outputs = outputs,
        inputs = header_info.transitive_headers,
        arguments = [args],
        executable = ctx.executable._mkdoc,
    )

# Generates a header that defines variables containing a representation of the
# contents of Doxygen comments for each class, function, etc. in the
# transitive headers of the given targets.
# @param targets Targets with header files that should have documentation
# strings generated.
# @param root_name Name of the root struct in generated file.
# @param hdr_subdir A specific directory name (e.g., "drake/multibody/tree")
# whose headers should be scraped; only headers in `target` that match this
# directory name exactly will be scraped.
# @param exclude_hdr_patterns Headers whose symbols should be ignored. Can be
# glob patterns.
generate_pybind_documentation_header = rule(
    attrs = {
        "targets": attr.label_list(
            mandatory = True,
        ),
        "_mkdoc": attr.label(
            default = Label("//tools/workspace/mkdoc_internal:mkdoc"),
            allow_files = True,
            cfg = "host",
            executable = True,
        ),
        "out": attr.output(mandatory = True),
        "root_name": attr.string(default = "pydrake_doc"),
        "hdr_subdir": attr.string(mandatory = True),
        "exclude_hdr_patterns": attr.string_list(),
    },
    implementation = _generate_pybind_documentation_header_impl,
    output_to_genfiles = True,
)
