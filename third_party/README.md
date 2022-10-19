The `/third_party` sub-directory contains software that is housed alongside
Drake, but was not authored by the Drake developers.  It typically has
different copyright ownership and licensing terms than the rest of Drake.

In general it is better to set up an external via a `tools/workspace`
directory whenever possible, as this makes it easier to follow the upstream
souce and customize more complex bazel rules.  Files should only be added here
if you need to fork and heavily modify a file to the point where its upstream
version is irrelevant.
