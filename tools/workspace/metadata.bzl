# -*- python -*-

def generate_repository_metadata(repository_ctx, **kwargs):
    """Helper function to archive load-phase metadata.  Writes a file named
    drake_repository_metadata.json to the root of an external repository.  The
    contents of the file will a JSON dict of the `kwargs`, with one additional
    dict entry of name="" using the repository_ctx.name.  This information can
    be loaded later using the tools/workspace/metadata.py library.
    """
    repository_ctx.file(
        "drake_repository_metadata.json",
        content = struct(
            name = repository_ctx.name,
            **kwargs
        ).to_json(),
        executable = False,
    )
