# -*- mode: python -*-
# vi: set ft=python :

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        archive = "dd-0.1.0-133-gd53b2231-qt-5.8.0-Darwin.tar.gz"
        sha256 = "7f5a352012d45e440278198d2b914016f035c6030552fe23d6154d33de7d41da"
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
                 attr=result.stderr)

        distro = [l.strip() for l in result.stdout.strip().split("\n")]
        distro = " ".join(distro)

        if distro == "Ubuntu 14.04":
            archive = "dd-0.1.0-133-gd53b223-qt-4.8.6-trusty-x86_64.tar.gz"
            sha256 = "bb04388913a98e26ba9dcbc57d0a97bc5bbb30590b5caedeab4f2ebed1cd9677"
        elif distro == "Ubuntu 16.04":
            archive = "dd-0.1.0-133-gd53b223-qt-5.5.1-xenial-x86_64.tar.gz"
            sha256 = "8713e17627c6ac7b169c0830481c61f6abe52d4025a8d1a7a914cd474f78a360"
        else:
            fail("Linux distribution is NOT supported", attr=distro)
    else:
        fail("Operating system is NOT supported", attr=repository_ctx.os.name)

    url = "https://d2mbb5ninhlpdu.cloudfront.net/director/{}".format(archive)
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(url, root_path, sha256=sha256)

    file_content = """
filegroup(
    name = "director",
    srcs = glob(["**/*"]),
    data = ["@vtk"],
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file("BUILD", content=file_content, executable=False)

director_repository = repository_rule(implementation = _impl)
