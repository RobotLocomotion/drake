def get_linux_distro(repository_ctx):
    """Get 'full' name of Linux distribution.

    This determines the 'full' name of the host platform Linux, i.e. the
    distribution name and version.

    The environment variable DRAKE_OVERRIDE_DISTRO may be set to override the
    computed value. Note that this is unsupported.
    """

    if "DRAKE_OVERRIDE_DISTRO" in repository_ctx.os.environ:
        distro = repository_ctx.os.environ["DRAKE_OVERRIDE_DISTRO"]
        print("Overriding built-in distribution detection " +
              "and pretending you are '%s' instead" % distro)
        return distro

    else:
        sed = repository_ctx.which("sed")
        if sed == None:
            fail("Could NOT determine Linux distribution information" +
                 "('sed' is missing?!)", sed)
        result = repository_ctx.execute([
            sed,
            "-n",
            "/^\(NAME\|VERSION_ID\)=/{s/[^=]*=//;s/\"//g;p}",
            "/etc/os-release"])

        if result.return_code != 0:
            fail("Could NOT determine Linux distribution information",
                 attr = result.stderr)

        distro = [l.strip() for l in result.stdout.strip().split("\n")]
        return " ".join(distro)
