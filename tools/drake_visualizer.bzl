# -*- mode: python -*-
# vi: set ft=python :

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        archive = "dv-0.1.0-168-gf2b4bb91-qt-5.9.1-mac.tar.gz"
        sha256 = "f3957532f661f74c9a2f4b5bf49d87f8634ea0771cac073d6708c24dd5b0feb8"  # noqa
    elif repository_ctx.os.name == "linux":
        sed = repository_ctx.which("sed")

        if not sed:
            fail("Could NOT determine Linux distribution information because" +
                 " sed is missing")

        result = repository_ctx.execute([
            sed,
            "-n",
            "/^\(NAME\|VERSION_ID\)=/{s/[^=]*=//;s/\"//g;p}",
            "/etc/os-release"])

        if result.return_code != 0:
            fail("Could NOT determine Linux distribution information",
                 attr = result.stderr)

        distro = [l.strip() for l in result.stdout.strip().split("\n")]
        distro = " ".join(distro)

        if distro == "Ubuntu 14.04":
            archive = "dv-0.1.0-168-gf2b4bb9-qt-4.8.6-trusty-x86_64.tar.gz"
            sha256 = "ccf1419f98b5b958fa667d7ea378a09e9d3fce186af9b94371a98165210b99e6"  # noqa
        elif distro == "Ubuntu 16.04":
            archive = "dv-0.1.0-168-gf2b4bb9-qt-5.5.1-xenial-x86_64.tar.gz"
            sha256 = "4b01e04b745db856c0dfeb5f4cc668fd5010df827aefdb1ec8ce1b6d96b54f82"  # noqa
        else:
            fail("Linux distribution is NOT supported", attr = distro)
    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    url = "https://d2mbb5ninhlpdu.cloudfront.net/director/{}".format(archive)
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(url, root_path, sha256 = sha256)

    file_content = """
filegroup(
    name = "drake_visualizer",
    srcs = glob([
        "lib/libPythonQt.*",
        "lib/libddApp.*",
        "lib/python2.7/dist-packages/director/**/*.py",
        "lib/python2.7/dist-packages/director/**/*.so",
        "lib/python2.7/dist-packages/urdf_parser_py/**/*.py",
    ]) + [
        "bin/drake-visualizer",
        "share/doc/director/LICENSE.txt",
    ],
    data = [
        "@lcm//:lcm-python",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_py",
        "@lcmtypes_robotlocomotion//:lcmtypes_robotlocomotion_py",
        "@vtk",
    ],
    visibility = ["//visibility:public"],
)

load("@drake//tools:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = [":drake_visualizer"],
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file("BUILD", content = file_content, executable = False)

drake_visualizer_repository = repository_rule(implementation = _impl)
