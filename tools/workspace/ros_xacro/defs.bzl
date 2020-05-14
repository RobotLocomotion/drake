# -*- python -*-

def _xacro_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.run(
        inputs = [ctx.file.src] + ctx.files.data,
        outputs = [out],
        executable = ctx.executable._tool,
        arguments = [
            "-o",
            out.path,
            ctx.file.src.path,
        ],
    )
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]

_xacro_rule = rule(
    attrs = {
        "src": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "data": attr.label_list(
            allow_files = True,
        ),
        "_tool": attr.label(
            default = "@ros_xacro//:xacro",
            cfg = "host",
            executable = True,
        ),
    },
    implementation = _xacro_impl,
)

def xacro_file(
        name,
        src = None,
        data = [],
        tags = [],
        visibility = None):
    """Runs xacro on a single input file, creating a single output file.

    Xacro is the ROS XML macro tool; http://wiki.ros.org/xacro.

    Args:
      name: The xml output file of this rule.
      src: The single xacro input file of this rule.
      data: Optional supplemental files required by the src file.
    """
    _xacro_rule(
        name = name,
        src = src,
        data = data,
        tags = tags,
        visibility = visibility,
    )

def xacro_filegroup(
        name,
        srcs = [],
        data = [],
        tags = [],
        visibility = None):
    """Runs xacro on several input files, creating a filegroup of the output.

    The output filenames will match the input filenames but with the ".xacro"
    suffix removed.

    Xacro is the ROS XML macro tool; http://wiki.ros.org/xacro.

    Args:
      name: The name of the filegroup label.
      srcs: The xacro input files of this rule.
      data: Optional supplemental files required by the srcs.
    """
    outs = []
    for src in srcs:
        if not src.endswith(".xacro"):
            fail("xacro_filegroup srcs should be named *.xacro not {}".format(
                src,
            ))
        out = src[:-6]
        outs.append(out)
        xacro_file(
            name = out,
            src = src,
            data = data,
            tags = tags,
            visibility = ["//visibility:private"],
        )
    native.filegroup(
        name = name,
        srcs = outs,
        visibility = visibility,
    )
