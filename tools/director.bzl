# -*- mode: python -*-
# vi: set ft=python :

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        archive = "dd-0.1.0-160-ga50a077-qt-5.8.0-Darwin.tar.gz"
        sha256 = "6873427eb417e03688e85ac59955777fda67f89781245f3146470c5156045691"  # noqa
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
            archive = "dd-0.1.0-160-ga50a077-qt-4.8.6-trusty-x86_64.tar.gz"
            sha256 = "7d8e5bf66648edffc8f003d0c0a19c80745cdf7b5c9a524069b041dd67c865d1"  # noqa
        elif distro == "Ubuntu 16.04":
            archive = "dd-0.1.0-160-ga50a077-qt-5.5.1-xenial-x86_64.tar.gz"
            sha256 = "c4dbd896454a293c3bb65761763ce0571e83dd81b6986e1458073d88a7ef55c7"  # noqa
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
    name = "director",
    srcs = glob(["**/*"]),
    data = ["@vtk"],
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file("BUILD", content = file_content, executable = False)

director_repository = repository_rule(implementation = _impl)
