# -*- python -*-

def impl(ctx):
  ctx.action(
      outputs=[ctx.outputs.out],
      executable=ctx.executable._script,
      arguments=[
        ctx.outputs.out.path
      ],
      use_default_shell_env=True,
  )

ccache_is_bad = rule(
    implementation=impl,
    outputs={
      "out": "ccache.txt",
    },
    attrs={
        "_script": attr.label(
            default=Label("//tools:complain_about_ccache.sh"),
            allow_single_file=True,
            executable=True,
            cfg="host",
        ),
    }
)
