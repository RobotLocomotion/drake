#!/bin/bash

set -euo pipefail

... etc ...

# Bootstrap the venv. The `setup` is required here (nothing else runs it), but
# the `sync` is not strictly requied (bazel will run it for us). We do it here
# anyway so that the user gets immediate, streaming feedback about problems
# with internet downloads, or other kinds of glitches. (When run from Bazel,
# stderr text is batched up until much later, not streamed.)
$(dirname $0)/../tools/workspace/venv/setup
$(dirname $0)/../tools/workspace/venv/sync

... etc ...
