/** @file
 Doxygen-only documentation for @ref multibody_parsing. */

/**
@defgroup multibody_parsing Parsing Models for Multibody Dynamics
@ingroup multibody

Drake's drake::multibody::Parser accepts model files written in either SDFormat
or URDF. In both formats, however, there are Drake-specific extensions and
Drake-specific limitations.

@anchor multibody_parsing_sdf
<h2>SDFormat Support</h2>
Drake supports SDFormat files following the specification at
http://sdformat.org/spec, As of this writing, the supported spec version is
1.9, but check also the Drake release notes for current status.

For Drake extensions to SDFormat files, see @ref multibody_parsing_drake_extensions.

<h3>SDFormat Standard Tags Not Supported by Drake</h3>
TODO(rpoyner-tri): reference section covering sdf unsupported

@anchor multibody_parsing_urdf
<h2>URDF Support</h2>
Drake supports URDF files as described here: http://wiki.ros.org/urdf/XML.
TODO(rpoyner-tri): is the URDF spec versioned?

For Drake extensions to URDF format files, see
@ref multibody_parsing_drake_extensions.

<h3>URDF Standard Tags Not Supported by Drake</h3>
TODO(rpoyner-tri): reference section covering urdf unsupported

@anchor multibody_parsing_drake_extensions
<h2>Drake Extensions</h2>

Drake extends both SDFormat and URDF in similar ways, to allow access to
Drakes-specific features. This section provides a compact reference to all Drake
extensions and their use in both formats.

TODO(rpoyner-tri): reference section covering all drake extensions

*/
