InstallInfo = provider()

#==============================================================================
#BEGIN internal helpers

#------------------------------------------------------------------------------
# Join paths without duplicating separators.
def _join_paths(*args):
  result = ""

  for part in args:
    if len(part) == 0:
      continue

    result += part
    if result[-1] != "/":
      result += "/"

  return result[:-1]

#------------------------------------------------------------------------------
# Remove prefix from path.
def ___remove_prefix(path, prefix):
  # If the prefix has more parts than the path, failure is certain.
  if len(prefix) > len(path):
    return None

  # Iterate over components to determine if a match exists.
  for n in range(len(prefix)):
    if prefix[n] == path[n]:
      continue
    elif prefix[n] == "*":
      continue
    else:
      return None

  return "/".join(path[len(prefix):])

def __remove_prefix(path, prefix):
  # Ignore trailing empty element (will happen if prefix string ends with "/").
  if len(prefix[-1]) == 0:
    prefix = prefix[:-1]

  # If the prefix has more parts than the path, failure is certain.
  if len(prefix) > len(path):
    return None

  # Iterate over components to determine if a match exists.
  for n in range(len(prefix)):
    if prefix[n] == path[n]:
      continue

    elif prefix[n] == "*":
      continue

    elif prefix[n] == "**":
      if n + 1 == len(prefix):
        return path[-1]

      for t in reversed(range(n, len(path) - (len(prefix) - n - 2))):
        x = ___remove_prefix(path[t:], prefix[n+1:])
        if x != None:
          return x

      return None

    else:
      return None

  return "/".join(path[len(prefix):])

def _remove_prefix(path, prefix):
  return __remove_prefix(path.split("/"), prefix.split("/"))

#------------------------------------------------------------------------------
# Compute output path for install action.
def _output_path(input_file, dest, strip_prefix, package_root):
  # Determine base path (will include workspace).
  i = input_file.path
  if input_file.is_source:
    i = _remove_prefix(i, package_root)
  else:
    i = _remove_prefix(i, _join_paths("bazel-out/*/*", package_root))

  # Possibly remove prefixes.
  for p in strip_prefix:
    o = _remove_prefix(i, p)
    if o != None:
      return o

  return i

#------------------------------------------------------------------------------
# Compute install actions for files.
def _install_actions(ctx, file_labels, dests, strip_prefix = []):
  actions = []
  package_root = _join_paths(ctx.label.workspace_root, ctx.label.package)

  # Iterate over files.
  for f in file_labels:
    for a in f.files:
      if type(dests) == "dict":
        dest = dests.get(a.extension, dests[None])
      else:
        dest = dests
      p = _output_path(a, dest, strip_prefix, package_root)
      actions.append(struct(src = a, dst = _join_paths(dest, p)))

  return actions

#------------------------------------------------------------------------------
# Compute install actions for a cc_library or cc_binary.
def _install_cc_actions(ctx, target):
  dests = {"a": "lib", "so": "lib", None: "bin"}
  return _install_actions(ctx, [target], dests)

#------------------------------------------------------------------------------
# Compute install actions for a java_library or java_binary.
def _install_java_actions(ctx, target):
  # TODO(mwoehlke-kitware): Implement this. Probably it mainly needs the logic
  # to pick install destinations appropriately.
  return []

#END internal helpers
#==============================================================================
#BEGIN rules

#------------------------------------------------------------------------------
# Generate information to install "stuff". "Stuff" can be library or binary
# targets, headers, or documentation files.
def _install_impl(ctx):
  actions = []

  # Collect install actions from dependencies.
  for d in ctx.attr.deps:
    actions += d.install_actions

  # Generate actions for docs and includes.
  actions += _install_actions(ctx, ctx.attr.docs, ctx.attr.doc_dest)
  actions += _install_actions(ctx, ctx.attr.hdrs, ctx.attr.hdr_dest,
                              ctx.attr.hdr_strip_prefix)

  for t in ctx.attr.targets:
    if hasattr(t, "cc"):
      actions += _install_cc_actions(ctx, t)
    elif hasattr(t, "java"):
      actions += _install_java_actions(ctx, t)

  return InstallInfo(install_actions = actions)

install = rule(
  implementation = _install_impl,
  attrs = {
    "deps": attr.label_list(providers = ["install_actions"]),
    "docs": attr.label_list(allow_files = True),
    "doc_dest": attr.string(default = "share/doc"),
    "hdrs": attr.label_list(allow_files = True),
    "hdr_dest": attr.string(default = "include"),
    "hdr_strip_prefix": attr.string_list(),
    "targets": attr.label_list(),
  },
)

#------------------------------------------------------------------------------
# Generate information to install files to specified destination.
def _install_files_impl(ctx):
  # Get path components
  dest = ctx.attr.dest
  strip_prefix = ctx.attr.strip_prefix

  # Generate actions
  actions = _install_actions(ctx, ctx.attr.files, dest, strip_prefix)

  # Return computed actions
  return InstallInfo(install_actions = actions)

install_files = rule(
  implementation = _install_files_impl,
  attrs = {
    "dest": attr.string(),
    "files": attr.label_list(allow_files = True),
    "strip_prefix": attr.string_list(),
  },
)

#------------------------------------------------------------------------------
# Generate install script from install rules.
def _install_driver_impl(ctx):
  pass
  # TODO(mwoehlke-kitware): Generate script from collected install actions, and
  # create rule to run the script as an argument to ctx.attr.driver. The run
  # rule should depend on all file artifacts in the collected install actions.
  # The run rule should also take a single argument, which is the install
  # destination.

install_driver = rule(
  implementation = _install_driver_impl,
  attrs = {
    "deps": attr.label_list(providers = ["install_actions"]),
    "driver": attr.label(allow_files = True),
  },
)

#END rules
