load("@drake//tools/workspace:os.bzl", "os_specific_alias_repository")

# How we build IPOPT depends on which platform we're on.
def ipopt_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "macOS default": [
                "ipopt=@ipopt_internal_pkgconfig//:ipopt_internal_pkgconfig",
                "install=@ipopt_internal_pkgconfig//:install",
            ],
            "Ubuntu default": [
                "ipopt=@ipopt_internal_fromsource//:ipopt",
                "install=@ipopt_internal_fromsource//:install",
            ],
        },
    )
