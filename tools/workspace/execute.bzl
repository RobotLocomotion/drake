# -*- python -*-

def execute_and_return(repo_ctx, command):
    """Runs the `command` (list) and returns a status value.  The return value
    is a struct with a field `error` that will be None on success or else a
    detailed message on command failure.
    """
    if "/" in command[0]:
        program = command[0]
    else:
        program = repo_ctx.which(command[0])
        if not program:
            error = "Could not find a program named '{}'".format(
                command[0])
            return struct(error = error)
    exec_result = repo_ctx.execute([program] + command[1:])
    if exec_result.return_code == 0:
        error = None
    else:
        error = "Failure running " + (
            " ".join(["'{}'".format(x) for x in command]))
        if exec_result.stdout:
            error += "\n" + exec_result.stdout
        if exec_result.stderr:
            error += "\n" + exec_result.stderr
    return struct(error = error)

def execute_or_fail(repo_ctx, command):
    """Runs the `command` (list) and immediately fails on any error."""
    result = execute_and_return(repo_ctx, command)
    if result.error:
        fail("Unable to complete setup for @{} repository: {}".format(
            repo_ctx.name, result.error))
