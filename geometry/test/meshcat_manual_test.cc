#include <future>

#include "drake/geometry/meshcat.h"

/**  To test, you must manually run `bazel run //geometry:meshcat_manual_test`.
It will print two URLs to console.  Navigating your browser to the first, you
should see that the normally blue meshcat background is not visible (the
background will look white).  In the second URL, you should see the default
meshcat view, but the grid that normally shows the ground plane is not visible.
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
  meshcat2.SetProperty("/Grid", "visible", false);

  // Sleep forever (we require the user to SIGINT to end the program).
  std::promise<void>().get_future().wait();

  return 0;
}
