# -*- python -*-

def path(repo_ctx, additional_search_paths = []):
    """Return the value of the PATH environment variable that would be used by
    the which() command."""
    search_paths = additional_search_paths

    # N.B. Ensure ${PATH} in each platform `tools/*.bazelrc` matches these
    # paths.
    if repo_ctx.os.name == "mac os x":
        search_paths = search_paths + ["/usr/local/bin"]
    search_paths = search_paths + ["/usr/bin", "/bin"]
    return ":".join(search_paths)

def which(repo_ctx, program, additional_search_paths = []):
    """Return the path of the given program or None if there is no such program
    in the PATH as defined by the path() function above. The value of the
    user's PATH environment variable is ignored.
    """
    exec_result = repo_ctx.execute(["which", program], environment = {
        "PATH": path(repo_ctx, additional_search_paths),
    })
    if exec_result.return_code != 0:
        return None
    return repo_ctx.path(exec_result.stdout.strip())

def execute_and_return(
        repo_ctx,
        command,
        additional_search_paths = [],
        **kwargs):
    """Runs the `command` (list) and returns a status value.  The return value
    is a struct with a field `error` that will be None on success or else a
    detailed message on command failure.
    """
    if "/" in str(command[0]):
        program = command[0]
    else:
        program = which(repo_ctx, command[0], additional_search_paths)
        if not program:
            error = "Could not find a program named '{}'".format(
                command[0],
            )
            return struct(error = error)
    exec_result = repo_ctx.execute([program] + command[1:], **kwargs)
    if exec_result.return_code == 0:
        error = None
    else:
        error = "Failure running " + (
            " ".join(["'{}'".format(x) for x in command])
        )
        if exec_result.stdout:
            error += "\n" + exec_result.stdout
        if exec_result.stderr:
            error += "\n" + exec_result.stderr
    return struct(
        error = error,
        stdout = exec_result.stdout,
    )

def execute_or_fail(repo_ctx, command, **kwargs):
    """Runs the `command` (list) and immediately fails on any error.
    Returns a struct with the stdout value."""
    result = execute_and_return(repo_ctx, command, **kwargs)
    if result.error:
        fail("Unable to complete setup for @{} repository: {}".format(
            repo_ctx.name,
            result.error,
        ))
    return result
