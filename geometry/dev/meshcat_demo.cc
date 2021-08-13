#include "drake/geometry/dev/meshcat.h"

/** This binary provides a simple demonstration of using Meshcat.  It serves as
a stand-in for the proper test suite that will come in the next PR, since it
requires it's own substantial changes to the build system.

To test, you must manually run `bazel run //geometry:meshcat_demo`.  It will
print two URLs to console.  Navigating your browser to the first, you should see
that the normally blue meshcat background is not visible (the background will
look white).  In the second URL, you should see the default meshcat view, but
observe that the checkbox in the controls gui next to `Grid` is unchecked.
*/

int main() {
  drake::geometry::Meshcat meshcat;

  // Note: this will only send one message to any new server.
  meshcat.SetProperty("/Background", "visible", false);
  meshcat.SetProperty("/Background", "visible", true);
  meshcat.SetProperty("/Background", "visible", false);

  // Demonstrate that we can construct multiple meshcats (and they will serve on
  // different ports).
  drake::geometry::Meshcat meshcat2;
  // Note: the grid doesn't actually disappear (bug in meshcat), but the
  // checkbox is unchecked.
  meshcat2.SetProperty("/Grid", "visible", false);

  // Effectively sleep forever.
  meshcat.join_websocket_thread();

  return 0;
}
