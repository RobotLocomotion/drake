def unwrap(repository_ctx):
  return getattr(repository_ctx, "repository_ctx", repository_ctx)

def wrapctx(repository_ctx, attr=None, os=None, name=None, root=None):
  if attr == None:
    attr = repository_ctx.attr
  if os == None:
    os = repository_ctx.os
  if name == None:
    name = repository_ctx.name
  if root == None:
    root = repo_root(repository_ctx)
  return struct(
      repository_ctx=unwrap(repository_ctx), attr=attr, os=os, name=name, root=root)

def repo_execute(ctx, args):
  return unwrap(ctx).execute(args)

def repo_root(ctx):
  if hasattr(ctx, "root"):
    return ctx.root
  return ctx.path("")

def repo_file(ctx, dest, contents, executable=True):
  return unwrap(ctx).file(repo_path(ctx, dest), contents, executable)

def repo_path(ctx, name):
  if type(name) == "string" and not name.startswith("/"):
    return repo_root(ctx).get_child(name)
  elif type(name) == "path":
    return name
  return unwrap(ctx).path(name)

def repo_symlink(ctx, src, dest):
  return unwrap(ctx).symlink(repo_path(ctx, src), repo_path(ctx, dest))

def repo_template(ctx, dest, template, repl, executable=True):
  return unwrap(ctx).template(
      repo_path(ctx, dest), repo_path(ctx, template), repl, executable)
