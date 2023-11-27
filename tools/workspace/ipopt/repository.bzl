load("//tools/workspace:os.bzl", "os_specific_alias_repository")

# How we build IPOPT depends on which platform we're on.
def ipopt_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "linux": [
                "ipopt=@ipopt_internal_fromsource//:ipopt",
                "install=@ipopt_internal_fromsource//:install",
            ],
            "osx": [
                "ipopt=@ipopt_internal_pkgconfig//:ipopt_internal_pkgconfig",
                "install=@ipopt_internal_pkgconfig//:install",
            ],
        },
    )
