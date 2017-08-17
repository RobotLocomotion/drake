# -*- mode: python -*-
# vi: set ft=python :

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        archive = "dd-0.1.0-133-g39640642-qt-5.9.1-Darwin.tar.gz"
        sha256 = "8679c1eb52c0216aafd01769f9aee51c467cd9dd38f0598e89640278f1409c6f"  # noqa
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
            archive = "dd-0.1.0-133-g3964064-qt-4.8.6-trusty-x86_64.tar.gz"
            sha256 = "171331401c520b4b631511b9c377a336e26ac71c5e0eee8134a4eec933dd42c8"  # noqa
        elif distro == "Ubuntu 16.04":
            archive = "dd-0.1.0-133-g3964064-qt-5.5.1-xenial-x86_64.tar.gz"
            sha256 = "f51e1c1992d884576d624b72659deb9fe942b0f887dd9561f63b445aa17d65a1"  # noqa
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
