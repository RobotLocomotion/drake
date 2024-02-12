load("//tools/workspace:os.bzl", "os_specific_alias_repository")

# Use this repository (@blas) in the deps of your target, instead of @libblas
# or @openblas. We use a different implementation depending on the operating
# system; this alias selects the correct implementation.
def blas_repository(name):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "linux": ["blas=@libblas"],
            "osx": ["blas=@openblas"],
        },
    )
