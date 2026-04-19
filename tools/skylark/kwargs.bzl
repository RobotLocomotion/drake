def amend(
        kwargs,
        option_name,
        *,
        default = None,
        prepend = None,
        append = None,
        update = None):
    """Changes one specific option_name in kwargs, using the mnemonically-named
    mutation operation(s) given by our optional arguments as follows:
    - `default` sets the option iff it is currently missing or set to None.
      It's an error is the option is already set to this exact value.
    - `prepend` adds a list to the front of a list-valued option.
    - `append` adds a list to the back of a list-valued option.
    - `update` updates a dict-valued option.
    If no optional arguments are provided, then kwargs is returned unchanged.

    As with most functions in the file, this takes a kwargs dict as the first
    argument and returns a modified copy of it.
    """
    if default != None:
        if kwargs.get(option_name) == default:
            fail(("Remove the {} argument; it is already the default").format(
                option_name,
            ))
        if kwargs.get(option_name) == None:
            kwargs[option_name] = default
    if prepend != None:
        kwargs[option_name] = prepend + kwargs.get(option_name, [])
    if append != None:
        kwargs[option_name] = kwargs.get(option_name, []) + append
    if update != None:
        item = kwargs.get(option_name, {})
        item.update(update)
        kwargs[option_name] = item
    return kwargs

def _bump_cpu_tag(kwargs, *, new_size):
    """Adds "cpu:{new_size}" tag if there are no other "cpu:*" tags specified,
    or if the existing "cpu:*" tag has a smaller size.

    As with most functions in the file, this takes a kwargs dict as the first
    argument and returns a modified copy of it.
    """
    add_new_tag = True
    tags = list(kwargs.get("tags", []))
    for tag in tags:
        if tag.startswith("cpu:"):
            existing_size = int(tag[4:])
            if existing_size < new_size:
                tags.remove(tag)
            else:
                add_new_tag = False
            break
    if add_new_tag:
        tags += ["cpu:{}".format(new_size)]
    kwargs["tags"] = tags
    return kwargs

def incorporate_rendering(kwargs, *, rendering):
    if rendering not in (True, False):
        fail("The 'rendering = ...' attribute should be a boolean.")
    if rendering:
        kwargs = amend(kwargs, "opt_out_conditions", append = [
            # Disable under LeakSanitizer and Valgrind Memcheck due to
            # driver-related leaks. For more information, see #7520.
            "//tools/lsan:enabled",
            "//tools/valgrind:enabled",
            # Similar to #7520, the GL vendor's libraries are not sufficiently
            # instrumented for compatibility with TSan.
            "//tools/tsan:enabled",
        ])
        kwargs = amend(kwargs, "tags", append = [
            # Mitigates driver-related issues when running under `bazel test`.
            # For more information, see #7004.
            "no-sandbox",
        ])
    return kwargs

def incorporate_display(kwargs, *, display):
    if display not in (True, False):
        fail("The 'display = ...' attribute should be a boolean.")
    if display:
        kwargs = amend(kwargs, "env_inherit", append = [
            "DISPLAY",
            "XAUTHORITY",
        ])
    return kwargs

def incorporate_num_threads(kwargs, *, num_threads):
    """Incorporates multi-threading directives (e.g., for OpenMP) into
    well-known Bazel args (i.e., tags=, env=).

    As with most functions in the file, this takes a kwargs dict as the first
    argument and returns a modified copy of it.
    """
    num_threads = num_threads or 1
    if num_threads < 1:
        fail("The num_threads must be strictly positive")
    if num_threads > 1:
        kwargs = _bump_cpu_tag(kwargs, new_size = num_threads)
        kwargs = amend(kwargs, "tags", append = ["omp"])
    kwargs = amend(kwargs, "env", update = {
        "DRAKE_NUM_THREADS": str(num_threads),
        "OMP_NUM_THREADS": str(num_threads),
        "OPENBLAS_NUM_THREADS": str(num_threads),
        "NUMEXPR_NUM_THREADS": str(num_threads),
        "MKL_NUM_THREADS": str(num_threads),
        "GUROBI_NUM_THREADS": str(num_threads),
    })
    return kwargs

def incorporate_allow_network(kwargs, *, allow_network):
    if allow_network == None or len(allow_network) == 0:
        allow_network = "meshcat"
    else:
        allow_network = ":".join(allow_network)
    kwargs = amend(kwargs, "env", update = {
        "DRAKE_ALLOW_NETWORK": allow_network,
    })
    return kwargs

def incorporate_test_weight_heuristics(kwargs):
    # kcov is only appropriate for small-sized unit tests. If a test needs a
    # shard_count or a special timeout, we assume it is not small.
    if "shard_count" in kwargs or "timeout" in kwargs:
        kwargs = amend(kwargs, "opt_out_conditions", append = [
            "//tools/kcov:enabled",
        ])
    return kwargs

def _construct_compatibility_select_expression(
        *,
        compatibles,
        incompatibles):
    terms = dict()
    terms.update({x: [] for x in compatibles})
    terms.update({x: ["@platforms//:incompatible"] for x in incompatibles})
    return select(terms)

def combine_conditions(
        *,
        opt_in_condition,
        opt_out_conditions):
    # Promote `None`s to default values.
    if opt_out_conditions == None:
        opt_out_conditions = []
    if opt_in_condition == None:
        opt_in_condition = "//conditions:default"

    # Choose where the vanilla case belongs.
    if opt_in_condition != "//conditions:default":
        opt_out_conditions.append("//conditions:default")

    # Build the select() dictionaries.
    positive = _construct_compatibility_select_expression(
        compatibles = [opt_in_condition],
        incompatibles = opt_out_conditions,
    )
    negative = _construct_compatibility_select_expression(
        compatibles = opt_out_conditions,
        incompatibles = [opt_in_condition],
    )

    return positive, negative
