load(":wrapped_ctx.bzl", "repo_file", "repo_template")

def error(message):
  return struct(value=None, error=message)

def success(value):
  return struct(value=value, error=None)

def globvalue(values, exclude=None, exclude_directories=True):
  keyword_arguments = {}
  if exclude:
    keyword_arguments["exclude"] = exclude
  if not exclude_directories:
    keyword_arguments["exclude_directories"] = exclude_directories
  return struct(context="list",
                function_name="glob",
                arguments=(values,),
                keyword_arguments=keyword_arguments)

def _fmt_key(key, context):
  if context == "list":
    return repr(["%{" + key + "}"])
  else:
    return repr("%{" + key + "}")

def _fmt_args(values):
  return ", ".join([repr(v) for v in values])

def _fmt_kwargs(values):
  return ", ".join(["%s=%s" % (k, repr(v)) for k, v in values.items()])

def _fmt_value(value):
  if type(value) == "struct" and hasattr(value, "function_name"):
    args = []
    if value.arguments:
      args += [_fmt_args(value.arguments)]
    if value.keyword_arguments:
      args += [_fmt_kwargs(value.keyword_arguments)]
    return "{}({})".format(value.function_name, ", ".join(args))
  else:
    return repr(value)

def _tmpl_entry(key, value):
  context = getattr(value, "context", type(value))
  return _fmt_key(key, context), _fmt_value(value)

def _tmpl_dict(values):
  tmpl = {}
  for item in values.items():
    key, value = _tmpl_entry(*item)
    tmpl[key] = value
  return tmpl

def write_build(repo_ctx, includes, defines, linkopts, srcs=None):
  if srcs == None:
    srcs = ["empty.cc"]
    # TODO(shahms): See if we can finally do away with this on OS X.
    repo_file(repo_ctx, "empty.cc", "", False)
  subs = _tmpl_dict({
      "name": repo_ctx.name,
      "srcs": srcs,
      "includes": includes,
      "defines": defines,
      "linkopts": linkopts,
  })
  repo_template(repo_ctx, "BUILD", repo_ctx.attr.build_file_template, subs)
