#pragma once

namespace drake {
namespace pydrake {
namespace pysystems {
namespace pylcm {

// LCM types should be bound on an as-needed basis in this module (unless it's
// a downstream project).
// TODO(eric.cousineau): Consider providing Starlark to codegen binding code.
void BindCppSerializers();

}  // namespace pylcm
}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
