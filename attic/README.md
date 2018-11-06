
Files in `attic/**` are scheduled for eventual deprecation, but in the meantime
they are separated into their own folder to de-clutter the Drake source tree.

When `#include`-ing files from this subtree, omit the `"attic/"` portion of the
path.  (Similarly, headers or data that are installed from this tree also 
will omit the `"attic/"` path element in the installed location.)

The C++ namespaces used by this files should NOT use an `attic` namespace, both
because the include paths do not mention the attic, and because the goal is to
avoid distrupting Drake users.

Drake's original RigidBodyTree class and its related code lives here.
The replacement implementation is MultibodyTree and its related code.

The name "attic" is inspried by the CVS source control system, e.g.,
https://docs.freebsd.org/info/cvs/cvs.info.Attic.html
