# -*- python -*-
# vi: set ft=python :

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
        "OMP_NUM_THREADS": str(num_threads),
        "OPENBLAS_NUM_THREADS": str(num_threads),
        "NUMEXPR_NUM_THREADS": str(num_threads),
        "MKL_NUM_THREADS": str(num_threads),
        "GUROBI_NUM_THREADS": str(num_threads),
    })
    return kwargs
