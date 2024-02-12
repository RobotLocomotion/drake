load("//tools/workspace:os.bzl", "os_specific_alias_repository")

# Use this repository (@lapack) in the deps of your target, instead of
# @liblapack or @openblas. We use a different implementations depending on the
# operating system; this alias exists to select the correct implementation.
def lapack_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "linux": ["lapack=@liblapack"],
            "osx": ["lapack=@openblas"],
        },
    )
