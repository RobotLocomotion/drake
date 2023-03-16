load(
    "@drake//tools/skylark:pathutils.bzl",
    "basename",
)
load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)
load(
    "@bazel_tools//tools/build_defs/repo:java.bzl",
    "java_import_external",
)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    # Create the /jar/BUILD.bazel file.
    build_content = """\
package(default_visibility = ["//visibility:public"])
"""
    if os_result.target in repo_ctx.attr.local_os_targets:
        is_local = True
        filename = basename(repo_ctx.attr.local_jar)
        repo_ctx.symlink(
            repo_ctx.attr.local_jar,
            "jar/{}".format(filename),
        )
        name = "jar"
        jars = [filename]
        build_content += "java_import(name = {name}, jars = {jars})\n".format(
            name = repr(name),
            jars = repr(jars),
        )
    else:
        is_local = False
        name = "jar"
        actual = "@_maven_{}//jar".format(repo_ctx.name)
        build_content += "alias(name = {name}, actual = {actual})\n".format(
            name = repr(name),
            actual = repr(actual),
        )
    repo_ctx.file("jar/BUILD.bazel", build_content)

    # Create the /BUILD.bazel file.
    name = "install"
    targets = [] if is_local else ["//jar"]
    install_content = """\
package(default_visibility = ["//visibility:public"])
load("@drake//tools/install:install.bzl", "install")
install(
    name = {name},
    targets = {targets},
    java_strip_prefix = ["**/"],
    allowed_externals = {targets},
)
"""
    repo_ctx.file("BUILD.bazel", install_content.format(
        name = repr(name),
        targets = repr(targets),
    ))

_internal_drake_java_import = repository_rule(
    attrs = {
        "licenses": attr.string_list(mandatory = True),
        "local_os_targets": attr.string_list(mandatory = True),
        "local_jar": attr.string(mandatory = True),
    },
    implementation = _impl,
)

def drake_java_import(
        name,
        *,
        licenses,
        local_os_targets,
        local_jar,
        maven_jar,
        maven_jar_sha256,
        mirrors):
    """A repository rule to bring in a Java dependency, either from the host's
    OS distribution, or else Maven. The list of local_os_targets indicates
    which distributions provide this jar; for those, the local_jar is the full
    path to the jar. Otherwise, the maven_jar will be used.
    """
    java_import_external(
        name = "_maven_{}".format(name),
        licenses = licenses,
        jar_urls = [
            x.format(fulljar = maven_jar)
            for x in mirrors.get("maven")
        ],
        jar_sha256 = maven_jar_sha256,
    )
    _internal_drake_java_import(
        name = name,
        licenses = licenses,
        local_os_targets = local_os_targets,
        local_jar = local_jar,
    )
