load(
    "//tools/skylark:kwargs.bzl",
    "amend",
    "incorporate_allow_network",
    "incorporate_display",
    "incorporate_num_threads",
)
load("//tools/skylark:sh.bzl", "sh_test")

def drake_sh_test(
        name,
        *,
        allow_network = None,
        display = False,
        num_threads = None,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param allow_network (optional, default is ["meshcat"])
        See drake/tools/skylark/README.md for details.

    @param display (optional, default is False)
        See drake/tools/skylark/README.md for details.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.

    By default, sets test size to "small" to indicate a unit test.
    """
    kwargs = incorporate_allow_network(kwargs, allow_network = allow_network)
    kwargs = incorporate_display(kwargs, display = display)
    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    kwargs = amend(kwargs, "size", default = "small")
    sh_test(
        name = name,
        **kwargs
    )
