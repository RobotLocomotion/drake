
Files in attic/** are scheduled for eventual deprecation, but in the meantime
they are separated into their own folder to de-clutter the Drake source tree.

When `#include`-ing files from this subtree, omit the `"attic/"` portion of the
path.  (Similarly, headers or data that are installed from this tree also strip
off the `"attic/"` path element.)

Drake's original RigidBodyTree class and its related code lives here.  The
replacement implementation is MultibodyTree and its related code.
