def generate_repository_metadata(repository_ctx, **kwargs):
    """Helper function to archive load-phase metadata. Writes a file named
    drake_repository_metadata.json to the root of an external repository.
    The contents of the file will be a JSON dict of the `kwargs`, with one
    additional dict entry of name="" based on the repository_ctx.name. This
    information can be loaded later using the tools/workspace/metadata.py
    library.
    """
    canonical_name = repository_ctx.name
    apparent_name = canonical_name.split("+")[-1]
    repository_ctx.file(
        "drake_repository_metadata.json",
        content = json.encode(struct(
            name = apparent_name,
            **kwargs
        )),
        executable = False,
    )
