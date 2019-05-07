# -*- python -*-

MainClassInfo = provider()

# Generate a launcher file to run installed java binaries
def _drake_java_binary_install_launcher_impl(ctx):
    classpath = ctx.attr.target[JavaInfo].compilation_info.runtime_classpath
    return [
        MainClassInfo(
            main_class = ctx.attr.main_class,
            classpath = classpath,
            filename = ctx.attr.filename,
            jvm_flags = ctx.attr.jvm_flags,
        ),
    ]

_drake_java_binary_install_launcher = rule(
    attrs = {
        "main_class": attr.string(mandatory = True),
        "filename": attr.string(mandatory = True),
        "target": attr.label(mandatory = True),
        "jvm_flags": attr.string_list(
            mandatory = False,
            default = [],
        ),
    },
    implementation = _drake_java_binary_install_launcher_impl,
)

"""Generate a launcher for java binary files.
"""

def drake_java_binary(
        name,
        main_class,
        **kwargs):
    """Creates a rule to declare a java binary and a MainClassInfo Provider

    The native java_binary creates a java launcher (shell script) that works in
    the build tree. However, a different launcher needs to be created to run
    the java binary in the install tree. This function creates a target
    `${name}-launcher` which contains a MainClassInfo provider. That provider
    will be used by the installer (install.bzl) to configure the installation
    script to generate a launcher script called `${name}-launcher.sh` at
    install time. That launcher can be renamed at install time to match the
    name of the java binary".
    """
    vkwargs = {
        key: value
        for key, value in kwargs.items()
        if key == "visibility" or key == "jvm_flags"
    }
    native.java_binary(
        name = name,
        main_class = main_class,
        **kwargs
    )
    launcher_name = name + "-launcher"
    _drake_java_binary_install_launcher(
        name = launcher_name,
        main_class = main_class,
        target = ":" + name,
        filename = launcher_name + ".sh",
        **vkwargs
    )
