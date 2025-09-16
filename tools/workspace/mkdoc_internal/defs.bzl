load("//tools/skylark:cc.bzl", "CcInfo")

def _collect_cc_header_info(targets):
    compile_flags = []
    transitive_headers_depsets = []
    package_headers_depsets = []
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

            transitive_headers_depset = compilation_context.headers
            transitive_headers_depsets.append(transitive_headers_depset)

            # Find all headers provided by the drake_cc_package_library, i.e.,
            # the set of transitively-available headers that exist in the same
            # Bazel package as the target.
            package_headers = [
                header
                for header in transitive_headers_depset.to_list()
                if (target.label.package == header.owner.package and
                    target.label.workspace_root == header.owner.workspace_root)
            ]

            # Remove headers that are duplicated as both a virtual include path
            # and a source path.  We'll use the virtual include path, since it
            # has a matching include path -- the source path does not.
            for header in list(package_headers):
                if header.path.startswith("bazel-out"):
                    continue

                # Confirm that the path is found elsewhere in virtual includes.
                if not any([
                    other
                    for other in package_headers
                    if (other != header and
                        other.path.endswith(header.path))
                ]):
                    fail("Header {} lacks a plausible include path".format(
                        header.path,
                    ))

                # Remove it.
                package_headers.remove(header)

            package_headers_depsets.append(depset(direct = package_headers))

    return struct(
        compile_flags = compile_flags,
        transitive_headers = depset(transitive = transitive_headers_depsets),
        package_headers = depset(transitive = package_headers_depsets),
    )

def _generate_pybind_documentation_header_impl(ctx):
    targets = _collect_cc_header_info(ctx.attr.targets)

    # N.B. We take this approach, rather than `target_exclude`, because it's
    # easier to add depsets together rather than subtract them.
    target_deps = _collect_cc_header_info(ctx.attr.target_deps)

    args = ctx.actions.args()

    # TODO(jamiesnape): Remove this line when #14034 is resolved.
    args.add("-DDRAKE_COMMON_SYMBOLIC_EXPRESSION_DETAIL_HEADER")
    args.add_all(
        targets.compile_flags + target_deps.compile_flags,
        uniquify = True,
    )
    outputs = [ctx.outputs.out]
    args.add("-output=" + ctx.outputs.out.path)
    args.add("-quiet")
    args.add("-root-name=" + ctx.attr.root_name)
    for p in ctx.attr.exclude_hdr_patterns:
        args.add("-exclude-hdr-patterns=" + p)
    args.add("-std=c++20")
    args.add("-w")

    # N.B. This is for `targets` only.
    args.add_all(targets.package_headers)

    ctx.actions.run(
        outputs = outputs,
        inputs = depset(transitive = [
            targets.transitive_headers,
            target_deps.transitive_headers,
        ]),
        arguments = [args],
        executable = ctx.executable._mkdoc,
    )

# Generates a header that defines variables containing a representation of the
# contents of Doxygen comments for each class, function, etc. in the
# transitive headers of the given targets.
# @param targets Targets with header files that should have documentation
# strings generated.
# @param target_deps Dependencies for `targets` (necessary for compilation /
# parsing), but should not have documentation generated.
# @param root_name Name of the root struct in generated file.
# @param exclude_hdr_patterns Headers whose symbols should be ignored. Can be
# glob patterns.
generate_pybind_documentation_header = rule(
    attrs = {
        "targets": attr.label_list(
            mandatory = True,
        ),
        "target_deps": attr.label_list(),
        "_mkdoc": attr.label(
            default = Label("//tools/workspace/mkdoc_internal:mkdoc"),
            allow_files = True,
            cfg = "host",
            executable = True,
        ),
        "out": attr.output(mandatory = True),
        "root_name": attr.string(default = "pydrake_doc"),
        "exclude_hdr_patterns": attr.string_list(),
    },
    implementation = _generate_pybind_documentation_header_impl,
    output_to_genfiles = True,
)
