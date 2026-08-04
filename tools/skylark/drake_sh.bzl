load(
    "//tools/skylark:kwargs.bzl",
    "amend",
    "combine_conditions",
    "incorporate_allow_network",
    "incorporate_display",
    "incorporate_num_threads",
    "incorporate_rendering",
)
load("//tools/skylark:sh.bzl", "sh_test")

def drake_sh_test(
        name,
        *,
        allow_network = None,
        display = False,
        num_threads = None,
        opt_in_condition = None,
        opt_out_conditions = None,
        rendering = False,
        **kwargs):
    """A wrapper to insert Drake-specific customizations.

    @param allow_network (optional, default is ["meshcat"])
        See drake/tools/skylark/README.md for details.

    @param display (optional, default is False)
        See drake/tools/skylark/README.md for details.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.

    @param opt_in_condition (optional, default is None)
        See drake/tools/skylark/README.md for details.

    @param opt_out_conditions (optional, default is None)
        See drake/tools/skylark/README.md for details.

    @param rendering (optional, default is False)
        See drake/tools/skylark/README.md for details.

    By default, sets test size to "small" to indicate a unit test.
    """
    kwargs = incorporate_allow_network(kwargs, allow_network = allow_network)
    kwargs = incorporate_display(kwargs, display = display)
    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    kwargs = incorporate_rendering(kwargs, rendering = rendering)
    kwargs = amend(kwargs, "size", default = "small")
    opt_out_conditions = (opt_out_conditions or []) + kwargs.pop("opt_out_conditions", [])
    target_compatible_with, _ = combine_conditions(
        name = name,
        opt_in_condition = opt_in_condition,
        opt_out_conditions = opt_out_conditions,
    )
    sh_test(
        name = name,
        target_compatible_with = target_compatible_with,
        **kwargs
    )
