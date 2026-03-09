#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/multibody/parsing/collision_filter_groups.h"
// #include "drake/multibody/parsing/detail_path_utils.h"
// #include "drake/multibody/parsing/model_directives.h"
// #include "drake/multibody/parsing/model_instance_info.h"
// #include "drake/multibody/parsing/package_map.h"
// #include "drake/multibody/parsing/parser.h"
// #include "drake/multibody/parsing/process_model_directives.h"
// #include "drake/multibody/parsing/scoped_names.h"

// Symbol: pydrake_doc_multibody_parsing
constexpr struct /* pydrake_doc_multibody_parsing */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::CollisionFilterGroups
      struct /* CollisionFilterGroups */ {
        // Source: drake/multibody/parsing/collision_filter_groups.h
        const char* doc =
R"""(This is storage for parsed collision filter groups and group pairs.
This data may be useful to users needing to compose further collision
filters in code, without having to restate the data already captured
in model files.

The contents of this object will be made up of names of collision
filter groups and bodies. By convention, the name strings are treated
as scoped names.

Note that this object enforces few invariants on the data. In the
expected workflow, the parser will add groups and exclusion pairs
found during parsing. The only condition checked here is that a group
with a given name is only added once.)""";
        // Symbol: drake::multibody::CollisionFilterGroups::AddExclusionPair
        struct /* AddExclusionPair */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc =
R"""(Adds an exclusion pair between two collision filter groups.

Parameter ``pair``:
    a pair of fully-qualified scoped names of groups.

Note:
    A pair can consist of the same name twice, which means the pair
    defines a rule where all members of the group exclude each other.
    Adding an already defined pair does nothing.)""";
        } AddExclusionPair;
        // Symbol: drake::multibody::CollisionFilterGroups::AddGroup
        struct /* AddGroup */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc =
R"""(Adds a new collision filter group.

Parameter ``name``:
    the fully-qualified scoped name of the group being defined.

Parameter ``members``:
    the fully-qualified scoped names of the member bodies.

Precondition:
    name is not already a defined group in this object.)""";
        } AddGroup;
        // Symbol: drake::multibody::CollisionFilterGroups::CollisionFilterGroups
        struct /* ctor */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::CollisionFilterGroups::empty
        struct /* empty */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc =
R"""(Returns:
    true iff both groups() and exclusion_pairs() are empty.)""";
        } empty;
        // Symbol: drake::multibody::CollisionFilterGroups::exclusion_pairs
        struct /* exclusion_pairs */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc =
R"""(Returns:
    the pairs stored by prior calls to AddExclusionPair().)""";
        } exclusion_pairs;
        // Symbol: drake::multibody::CollisionFilterGroups::groups
        struct /* groups */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc =
R"""(Returns:
    the groups stored by prior calls to AddGroup().)""";
        } groups;
        // Symbol: drake::multibody::CollisionFilterGroups::to_string
        struct /* to_string */ {
          // Source: drake/multibody/parsing/collision_filter_groups.h
          const char* doc =
R"""(Returns:
    a multi-line human-readable representation of this' contents.)""";
        } to_string;
      } CollisionFilterGroups;
      // Symbol: drake::multibody::PackageMap
      struct /* PackageMap */ {
        // Source: drake/multibody/parsing/package_map.h
        const char* doc =
R"""(Maps ROS package names to their full path on the local file system. It
is used by the SDF and URDF parsers when parsing files that reference
ROS packages for resources like mesh files. This class can also
download remote packages from the internet on an as-needed basis via
AddRemote().)""";
        // Symbol: drake::multibody::PackageMap::Add
        struct /* Add */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Adds package ``package_name`` and its path, ``package_path``. Throws
if ``package_name`` is already present in this PackageMap with a
different path, or if ``package_path`` does not exist.)""";
        } Add;
        // Symbol: drake::multibody::PackageMap::AddMap
        struct /* AddMap */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Adds all packages from ``other_map`` into ``this``. Throws if
``other`` contains a package with the same ``package_name`` as one
already in this map but with incompatible details (e.g., a different
local path).)""";
        } AddMap;
        // Symbol: drake::multibody::PackageMap::AddPackageXml
        struct /* AddPackageXml */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Adds an entry into this PackageMap for the given ``package.xml``
filename. Throws if ``filename`` does not exist or its embedded name
already exists in this map.)""";
        } AddPackageXml;
        // Symbol: drake::multibody::PackageMap::AddRemote
        struct /* AddRemote */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Adds an entry into this PackageMap for the given ``package_name``,
which will be downloaded from the internet (with local caching). The
data will not be downloaded until necessary, i.e., when GetPath() is
first called for the ``package_name``. Throws if the ``package_name``
or ``params`` are invalid. Downloading requires a valid
``/usr/bin/python3`` interpreter, which will be invoked as a
subprocess.

See allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
option to disable network fetching. AddRemote may still be used even
with network fetching disabled -- in that case, the urls must contain
a "file://" URL or the download cache must already contain a
previously-downloaded copy of the package (with the same sha256
checksum).)""";
        } AddRemote;
        // Symbol: drake::multibody::PackageMap::Contains
        struct /* Contains */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Returns true if and only if this PackageMap contains ``package_name``.)""";
        } Contains;
        // Symbol: drake::multibody::PackageMap::GetDeprecated
        struct /* GetDeprecated */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Returns the deprecation message for package ``package_name`` if it has
been set as deprecated. A value of std∷nullopt implies no deprecation.
Aborts if no package named ``package_name`` exists in this PackageMap.)""";
        } GetDeprecated;
        // Symbol: drake::multibody::PackageMap::GetPackageNames
        struct /* GetPackageNames */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Returns the package names in this PackageMap. The order of package
names returned is unspecified.)""";
        } GetPackageNames;
        // Symbol: drake::multibody::PackageMap::GetPath
        struct /* GetPath */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Obtains the path associated with package ``package_name``. Aborts if
no package named ``package_name`` exists in this PackageMap.

Parameter ``deprecated_message``:
    When passed as nullptr (its default value), then in case the
    ``package_name`` is deprecated a deprecation message will be
    logged. When passed as non-nullptr the deprecation message will be
    output into this argument and will not be logged; if the
    ``package_name`` is not deprecated, the message will be set to
    nullopt.)""";
        } GetPath;
        // Symbol: drake::multibody::PackageMap::MakeEmpty
        struct /* MakeEmpty */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(A factory method that initializes an empty map.)""";
        } MakeEmpty;
        // Symbol: drake::multibody::PackageMap::PackageMap
        struct /* ctor */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(A constructor that initializes a default map containing only the
top-level ``drake`` manifest. See PackageMap∷MakeEmpty() to create an
empty map.)""";
        } ctor;
        // Symbol: drake::multibody::PackageMap::PopulateFromEnvironment
        struct /* PopulateFromEnvironment */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Obtains one or more paths from environment variable
``environment_variable``. Crawls downward through the directory
tree(s) starting from the path(s) searching for ``package.xml`` files.
For each of these files, this method adds a new entry into this
PackageMap where the key is the package name as specified within
``package.xml`` and the value is the path to the ``package.xml`` file.
Multiple paths can be specified by separating them using the ':'
symbol. For example, the environment variable can contain [path
1]:[path 2]:[path 3] to search three different paths.

If a package already known by the PackageMap is found again with a
conflicting path, a warning is logged and the original path is kept.

If a path does not exist or is unreadable, a warning is logged.

Warning:
    This function must not be used when populating manifests from the
    ROS_PACKAGE_PATH environment variable. It will throw an exception
    when that is attempted. Instead, use PopulateFromRosPackagePath().)""";
        } PopulateFromEnvironment;
        // Symbol: drake::multibody::PackageMap::PopulateFromFolder
        struct /* PopulateFromFolder */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Crawls down the directory tree starting at ``path`` searching for
directories containing the file ``package.xml``. For each of these
directories, this method adds a new entry into this PackageMap where
the key is the package name as specified within ``package.xml`` and
the directory's path is the value. If a package already known by the
PackageMap is found again with a conflicting path, a warning is logged
and the original path is kept. If the path does not exist or is
unreadable, a warning is logged.)""";
        } PopulateFromFolder;
        // Symbol: drake::multibody::PackageMap::PopulateFromRosPackagePath
        struct /* PopulateFromRosPackagePath */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Obtains one or more paths from the ROS_PACKAGE_PATH environment
variable. Semantics are similar to PopulateFromEnvironment(), except
that ROS-style crawl termination semantics are enabled, which means
that subdirectories of already-identified packages are not searched,
and neither are directories which contain any of the following marker
files: - AMENT_IGNORE - CATKIN_IGNORE - COLCON_IGNORE)""";
        } PopulateFromRosPackagePath;
        // Symbol: drake::multibody::PackageMap::RemoteParams
        struct /* RemoteParams */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc = R"""(Parameters used for AddRemote().)""";
          // Symbol: drake::multibody::PackageMap::RemoteParams::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/package_map.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::PackageMap::RemoteParams::ToJson
          struct /* ToJson */ {
            // Source: drake/multibody/parsing/package_map.h
            const char* doc =
R"""(Returns the JSON serialization of these params.)""";
          } ToJson;
          // Symbol: drake::multibody::PackageMap::RemoteParams::archive_type
          struct /* archive_type */ {
            // Source: drake/multibody/parsing/package_map.h
            const char* doc =
R"""((Optional) The archive type of the downloaded file. Valid options are
"zip", "tar", "gztar", "bztar", or "xztar". By default, the archive
type is determined from the file extension of the URL; in case the URL
has no filename extension, you should explicitly specify one here.)""";
          } archive_type;
          // Symbol: drake::multibody::PackageMap::RemoteParams::sha256
          struct /* sha256 */ {
            // Source: drake/multibody/parsing/package_map.h
            const char* doc =
R"""(The cryptographic checksum of the file to be downloaded, as a
64-character hexadecimal string.)""";
          } sha256;
          // Symbol: drake::multibody::PackageMap::RemoteParams::strip_prefix
          struct /* strip_prefix */ {
            // Source: drake/multibody/parsing/package_map.h
            const char* doc =
R"""((Optional) A directory prefix to remove from the extracted files. In
many cases, an archive will prefix all filenames with something like
"package-v1.2.3/" so that it extracts into a convenient directory.
This option will discard that common prefix when extracting the
archive for the PackageMap. It is an error if the archive does not
contain any diectory with this prefix, but if there are files outside
of this directory they will be silently discarded.)""";
          } strip_prefix;
          // Symbol: drake::multibody::PackageMap::RemoteParams::urls
          struct /* urls */ {
            // Source: drake/multibody/parsing/package_map.h
            const char* doc =
R"""(The list of remote URLs for this resource. The urls are used in the
other they appear here, so preferred mirror(s) should come first.
Valid methods are "http://" or "https://" or "file://".)""";
          } urls;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("archive_type", archive_type.doc),
              std::make_pair("sha256", sha256.doc),
              std::make_pair("strip_prefix", strip_prefix.doc),
              std::make_pair("urls", urls.doc),
            };
          }
        } RemoteParams;
        // Symbol: drake::multibody::PackageMap::Remove
        struct /* Remove */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Removes package ``package_name`` and its previously added path. Throws
if ``package_name`` is not present in this PackageMap.)""";
        } Remove;
        // Symbol: drake::multibody::PackageMap::ResolveUrl
        struct /* ResolveUrl */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Returns a resolved path for ``url``. URL schemes are either
``file://`` for local files or ``package://`` (or ``model://``).

Raises:
    RuntimeError if the url cannot be resolved.)""";
        } ResolveUrl;
        // Symbol: drake::multibody::PackageMap::SetDeprecated
        struct /* SetDeprecated */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Sets or clears the deprecation message for package ``package_name``. A
``deprecated_message`` value of std∷nullopt implies no deprecation.
Aborts if no package named ``package_name`` exists in this PackageMap.)""";
        } SetDeprecated;
        // Symbol: drake::multibody::PackageMap::size
        struct /* size */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc =
R"""(Returns the number of entries in this PackageMap.)""";
        } size;
        // Symbol: drake::multibody::PackageMap::to_string
        struct /* to_string */ {
          // Source: drake/multibody/parsing/package_map.h
          const char* doc = R"""()""";
        } to_string;
      } PackageMap;
      // Symbol: drake::multibody::Parser
      struct /* Parser */ {
        // Source: drake/multibody/parsing/parser.h
        const char* doc =
R"""(Parses model description input into a MultibodyPlant and (optionally)
a SceneGraph. A variety of input formats are supported, and are
recognized by filename suffixes:

File format | Filename suffix ------------------------ |
--------------- URDF | ".urdf" SDFormat | ".sdf" MJCF (Mujoco XML) |
".xml" Drake Model Directives | ".dmd.yaml" Wavefront OBJ | ".obj"

The output of parsing is one or more model instances added to the
MultibodyPlant provided to the parser at construction.

For an introductory tutorial about parsing, see the <a
href="https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/authoringmultibodysimulation-8159e3be6dc048a6a8ab6876017def4b">Authoring
a Multibody Simulation</a> page.

SDFormat files may contain multiple ``<model>`` elements. New model
instances will be added to ``plant`` for each ``<model>`` tag in the
file.

Note:
    Adding multiple root-level models, i.e, ``<model>`s directly under
    `<sdf>``, is deprecated. If you need multiple models in a single
    file, please use an SDFormat world tag.

URDF files contain a single ``<robot>`` element. Only a single model
instance will be added to ``plant``.

MJCF (MuJoCo XML) files typically contain many bodies, they will all
be added as a single model instance in the ``plant``.

Drake Model Directives are only available via AddModels or
AddModelsFromString. The single-model methods (AddModelFromFile,
AddModelFromString) cannot load model directives.

OBJ files will infer a model with a single body from the geometry. The
OBJ file must contain a *single* object (in the OBJ-file sense). The
body's mass properties are computed based on uniform distribution of
material in the enclosed volume of the mesh (with the approximate
density of water: 1000 kg/m³). If the mesh is not a closed manifold,
this can produce unexpected results. The spatial inertia of the body
is measured at the body frame's origin. The body's frame is coincident
and fixed with the frame the mesh's vertices are measured and
expressed in. The mesh's vertices are assumed to be measured in units
of *meters*.

The name of the model and body are determined according to the
following prioritized protocol:

- The non-empty ``model_name``, if given (e.g., in
AddModelFromFile()). - If the object is named in the obj file, that
object name is used. - Otherwise, the base name of the file name is
used (i.e., the file name with the prefixed directory and extension
removed).

If the underlying plant is registered with a SceneGraph instance, the
mesh will also be used for all three roles: illustration, perception,
and proximity.

Warning:
    AddModelsFromString() cannot be passed OBJ file contents yet.

For more documentation of Drake-specific treatment of these input
formats, see multibody_parsing.

When parsing literal quantities, Parser assumes SI units and radians
in the absence of units specified by the format itself. This includes
the literals in the explicitly specified files as well as referenced
files such as OBJ or other data file formats.

MultibodyPlant requires that model instances have unique names. To
support loading multiple instances of the same model file(s) into a
plant, Parser offers a few different strategies.

Parser has constructors that take a model name prefix, which gets
applied to all models loaded with that Parser instance. The resulting
workflow makes multiple parsers to build models for a single plant:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Parser left_parser(plant, "left");
     Parser right_parser(plant, "right");
     left_parser.AddModels(arm_model);  // "left∷arm"
     right_parser.AddModels(arm_model);  // "right∷arm"
     left_parser.AddModels(gripper_model);  // "left∷gripper"
     right_parser.AddModels(gripper_model);  // "right∷gripper"

.. raw:: html

    </details>

For situations where it is convenient to load a model many times,
Parser offers optional auto-renaming. When auto-renaming is enabled,
name collisions will be resolved by adding a subscript to the name.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Parser parser(plant);
     parser.SetAutoRenaming(true);
     // Subscripts are compact, and start at 1.
     parser.AddModels(rock);  // "rock"
     parser.AddModels(rock);  // "rock_1"
     parser.AddModels(rock);  // "rock_2"
     // Subscripts of different base names are independent.
     parser.AddModels(stone);  // "stone"
     parser.AddModels(stone);  // "stone_1"
     parser.AddModels(stone);  // "stone_2"

.. raw:: html

    </details>

(Advanced) In the rare case where the user is parsing into a
MultibodyPlant and SceneGraph but has created them one at a time
instead of using the more convenient AddMultibodyPlant() or
AddMultibodyPlantSceneGraph() functions, the Parser constructors
accept an optional SceneGraph pointer to specify which SceneGraph to
parse into. If it is provided and non-null and the MultibodyPlant is
not registered as a source, the Parser will perform the SceneGraph
registration into the given plant. We describe this option only for
completeness; we strongly discourage anyone from taking advantage of
this feature.)""";
        // Symbol: drake::multibody::Parser::AddModels
        struct /* AddModels */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Parses the input file named in ``file_name`` and adds all of its
model(s) to ``plant``.

Parameter ``file_name``:
    The name of the file to be parsed. The file type will be inferred
    from the extension.

Returns:
    The set of model instance indices for the newly added models,
    including nested models.

Raises:
    RuntimeError in case of errors.)""";
        } AddModels;
        // Symbol: drake::multibody::Parser::AddModelsFromString
        struct /* AddModelsFromString */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Provides same functionality as AddModels, but instead parses the model
description text data via ``file_contents`` with format dictated by
``file_type``.

Parameter ``file_contents``:
    The model data to be parsed.

Parameter ``file_type``:
    The data format; must be one of the filename suffixes listed
    above, *without* the leading dot (.).

Returns:
    The set of model instance indices for the newly added models,
    including nested models.

Raises:
    RuntimeError in case of errors.)""";
        } AddModelsFromString;
        // Symbol: drake::multibody::Parser::AddModelsFromUrl
        struct /* AddModelsFromUrl */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Parses the input file named in ``url`` and adds all of its model(s) to
``plant``. The allowed URL schemes are either ``file://`` for local
files or ``package://`` (or ``model://``) to use this Parser's
``package_map()``.

Parameter ``url``:
    The file to be parsed. The file type will be inferred from the
    extension.

Returns:
    The set of model instance indices for the newly added models,
    including nested models.

Raises:
    RuntimeError in case of errors.)""";
        } AddModelsFromUrl;
        // Symbol: drake::multibody::Parser::GetAutoRenaming
        struct /* GetAutoRenaming */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Get the current state of auto-renaming.

See also:
    the Parser class documentation for more detail.)""";
        } GetAutoRenaming;
        // Symbol: drake::multibody::Parser::GetCollisionFilterGroups
        struct /* GetCollisionFilterGroups */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Gets the accumulated set of collision filter definitions seen by this
parser.

There are two kinds of names in the returned data: group names and
body names. Both may occur within scoped names indicating the model
instance where they are defined. Note that the model instance names
used in the returned data will reflect the current names in plant() at
the time this accessor is called (see
MultibodyPlant∷RenameModelInstance()), but the local group and body
names will be the names seen during parsing.)""";
        } GetCollisionFilterGroups;
        // Symbol: drake::multibody::Parser::Parser
        struct /* ctor */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc_4args_builder_plant_scene_graph_model_name_prefix =
R"""(Create a Parser given a DiagramBuilder (and optionally plant and
scene_graph). If specified, the resulting parser will apply
``model_name_prefix`` to the names of any models parsed. ``builder``
may be nullptr, but then you must specify a ``plant``.

Precondition:
    Either the given ``builder`` contains a MultibodyPlant system
    named "plant" or else the provided ``plant`` is non-null.

Precondition:
    If both ``builder`` and ``plant`` are specified, then plant ∈
    builder.GetSystems()

Parameter ``scene_graph``:
    A pointer to a mutable SceneGraph object used for geometry
    registration (either to model visual or contact geometry). May be
    nullptr.

Precondition:
    If both ``builder`` and ``scene_graph`` are specified, then
    scene_graph ∈ builder.GetSystems().)""";
          // Source: drake/multibody/parsing/parser.h
          const char* doc_2args_plant_scene_graph =
R"""(Creates a Parser that adds models to the given plant and (optionally)
scene_graph.

Parameter ``plant``:
    A pointer to a mutable MultibodyPlant object to which parsed
    model(s) will be added; ``plant->is_finalized()`` must remain
    ``False`` for as long as the ``plant`` is in use by ``this``.

Parameter ``scene_graph``:
    A pointer to a mutable SceneGraph object used for geometry
    registration (either to model visual or contact geometry). May be
    nullptr.)""";
          // Source: drake/multibody/parsing/parser.h
          const char* doc_3args_plant_scene_graph_model_name_prefix =
R"""(Creates a Parser that adds models to the given plant and scene_graph.
The resulting parser will apply ``model_name_prefix`` to the names of
any models parsed.

Parameter ``plant``:
    A pointer to a mutable MultibodyPlant object to which parsed
    model(s) will be added; ``plant->is_finalized()`` must remain
    ``False`` for as long as the ``plant`` is in use by ``this``.

Parameter ``scene_graph``:
    A pointer to a mutable SceneGraph object used for geometry
    registration (either to model visual or contact geometry). May be
    nullptr.

Parameter ``model_name_prefix``:
    A string that will be added as a scoped name prefix to the names
    of any models loaded by this parser; when empty, no scoping will
    be added.)""";
          // Source: drake/multibody/parsing/parser.h
          const char* doc_2args_plant_model_name_prefix =
R"""(Creates a Parser that adds models to the given plant and scene_graph.
The resulting parser will apply ``model_name_prefix`` to the names of
any models parsed.

Parameter ``plant``:
    A pointer to a mutable MultibodyPlant object to which parsed
    model(s) will be added; ``plant->is_finalized()`` must remain
    ``False`` for as long as the ``plant`` is in use by ``this``.

Parameter ``model_name_prefix``:
    A string that will be added as a scoped name prefix to the names
    of any models loaded by this parser; when empty, no scoping will
    be added.)""";
        } ctor;
        // Symbol: drake::multibody::Parser::SetAutoRenaming
        struct /* SetAutoRenaming */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Enable or disable auto-renaming. It is disabled by default.

See also:
    the Parser class documentation for more detail.)""";
        } SetAutoRenaming;
        // Symbol: drake::multibody::Parser::SetStrictParsing
        struct /* SetStrictParsing */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Cause all subsequent Add*Model*() operations to use strict parsing;
warnings will be treated as errors.)""";
        } SetStrictParsing;
        // Symbol: drake::multibody::Parser::builder
        struct /* builder */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Gets a mutable pointer to the DiagramBuilder that will be modified by
this parser, or nullptr if this parser does not have a DiagramBuilder.)""";
        } builder;
        // Symbol: drake::multibody::Parser::package_map
        struct /* package_map */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Gets a mutable reference to the PackageMap used by this parser.)""";
        } package_map;
        // Symbol: drake::multibody::Parser::plant
        struct /* plant */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Gets a mutable reference to the plant that will be modified by this
parser.)""";
        } plant;
        // Symbol: drake::multibody::Parser::scene_graph
        struct /* scene_graph */ {
          // Source: drake/multibody/parsing/parser.h
          const char* doc =
R"""(Gets a mutable pointer to the SceneGraph that will be modified by this
parser, or nullptr if this parser does not have a SceneGraph.)""";
        } scene_graph;
      } Parser;
      // Symbol: drake::multibody::parsing
      struct /* parsing */ {
        // Symbol: drake::multibody::parsing::AddCollisionFilterGroup
        struct /* AddCollisionFilterGroup */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Directive to add a collision filter group. This directive is analogous
to tag_drake_collision_filter_group in XML model formats.)""";
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::ignored_collision_filter_groups
          struct /* ignored_collision_filter_groups */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Names of groups against which to ignore collisions. If another group
is named, collisions between this group and that group will be
ignored. If this group is named, collisions within this group will be
ignored. Names may be scoped and refer to other groups defined
elsewhere in this file or transitively included directives or model
files. This data is analogous to a sequence of
tag_drake_ignored_collision_filter_group in XML model formats.)""";
          } ignored_collision_filter_groups;
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::member_groups
          struct /* member_groups */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Names of groups to add en masse as members of the group. May be scoped
and refer to bodies of already added models. This data is analogous to
a sequence of tag_drake_member_group in XML model formats.)""";
          } member_groups;
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::members
          struct /* members */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Names of members of the group. May be scoped and refer to bodies of
already added models. This data is analogous to a sequence of
tag_drake_member in XML model formats.)""";
          } members;
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::model_namespace
          struct /* model_namespace */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Optional model namespace. Allows ``name`` to be reused between models
and lets you use the scoped name in
``ignored_collision_filter_groups``.)""";
          } model_namespace;
          // Symbol: drake::multibody::parsing::AddCollisionFilterGroup::name
          struct /* name */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Name of group to be added. This is an unscoped name, and must be
unique either globally or within its specified model namespace.)""";
          } name;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("ignored_collision_filter_groups", ignored_collision_filter_groups.doc),
              std::make_pair("member_groups", member_groups.doc),
              std::make_pair("members", members.doc),
              std::make_pair("model_namespace", model_namespace.doc),
              std::make_pair("name", name.doc),
            };
          }
        } AddCollisionFilterGroup;
        // Symbol: drake::multibody::parsing::AddDirectives
        struct /* AddDirectives */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Directive to incorporate another model directives file, optionally
with its elements prefixed with a namespace.)""";
          // Symbol: drake::multibody::parsing::AddDirectives::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::AddDirectives::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::AddDirectives::file
          struct /* file */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(The ``package://`` URI of the file to add.)""";
          } file;
          // Symbol: drake::multibody::parsing::AddDirectives::model_namespace
          struct /* model_namespace */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Namespaces base model instance for processing directive files. Affects
scoping (i.e. the following members): - AddModel∷name -
AddModelInstance∷name - AddFrame∷name - AddWeld∷parent - AddWeld∷child
- AddFrame∷X_PF∷base_frame - AddCollisionFilterGroup∷name -
AddCollisionFilterGroup∷members -
AddCollisionFilterGroup∷ignored_colllision_filter_groups -
AddDirectives∷model_namespace See ``README.md`` for example references
and namespacing.)""";
          } model_namespace;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("file", file.doc),
              std::make_pair("model_namespace", model_namespace.doc),
            };
          }
        } AddDirectives;
        // Symbol: drake::multibody::parsing::AddFrame
        struct /* AddFrame */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Directive to add a Frame to the scene. The added frame must have a
name and a transform with a base frame and offset.)""";
          // Symbol: drake::multibody::parsing::AddFrame::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::AddFrame::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::AddFrame::X_PF
          struct /* X_PF */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Pose of frame to be added, ``F``, w.r.t. parent frame ``P`` (as
defined by ``X_PF.base_frame``).)""";
          } X_PF;
          // Symbol: drake::multibody::parsing::AddFrame::name
          struct /* name */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Name of frame to be added. If scope is specified, will override model
instance; otherwise, will use `X_PF.base_frame`s instance.)""";
          } name;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("X_PF", X_PF.doc),
              std::make_pair("name", name.doc),
            };
          }
        } AddFrame;
        // Symbol: drake::multibody::parsing::AddModel
        struct /* AddModel */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Directive to add a model from a URDF or SDFormat file to a scene,
using a given name for the added instance.)""";
          // Symbol: drake::multibody::parsing::AddModel::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::AddModel::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::AddModel::default_free_body_pose
          struct /* default_free_body_pose */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Map of body_name or frame_name => default free body pose. The name
must be a name within the scope of the model added by this directive.
The name must not be scoped (i.e., no "foo∷link", just "link"). If the
name is empty, then the posed frame will be the body frame of the
model's sole body (and if the model has >1 body then it is an error).

For deformable bodies, the body pose defines the world pose of the
reference configuration, i.e. the undeformed geometry of the
deformable body will have this pose in the world frame.

However, the schema∷Transform associated with that named body/frame
can define a ``base_frame`` referring to any frame that has been added
prior to or including this declaration. The named frame must *always*
be a scoped name, even if its part of the model added by this
directive.

Warning:
    there are two important implications for the named frame if the
    transform's ``base_frame`` is not the world (explicitly or
    implicitly by omission):

1. The named body will *not* be considered a "floating base" body (see
mbp_working_with_free_bodies "Working with free bodies"). Calls to
MultibodyPlant∷SetDefaultFreeBodyPose() will have no effect on an
allocated context. If you want to change its default pose after adding
the model, you need to acquire the body's joint and set the new
default pose on the joint directly. Note: what you will *actually* be
posing is the *named* frame. If it's the name of the body, you will be
posing the body. If it's a frame affixed to the body frame, you will
be posing the fixed frame (with the body offset based on the
relationship between the two frames). 2. The body associated with the
named frame will have a six-dof joint between itself and the body
associated with the transform's ``base_frame``. When interpreting the
qs for the "named" body, it is the six-dof pose of the body measured
and expressed in the parent frame (transform's ``base_frame``). This
is true whether setting the position values in the resulting joint
directly or using the mbp_working_with_free_bodies "MultibodyPlant
free body APIs". 3. If the body is deformable, non-world
``base_frame`` is an error.

Warning:
    There should not already be a joint in the model between the two
    bodies implied by the named frames.)""";
          } default_free_body_pose;
          // Symbol: drake::multibody::parsing::AddModel::default_joint_positions
          struct /* default_joint_positions */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Map of joint_name => default position vector. Each joint name must be
a name within the scope of the model added by this directive. The name
must not contains *this* model's scoped name (nor that of any
previously added model).)""";
          } default_joint_positions;
          // Symbol: drake::multibody::parsing::AddModel::file
          struct /* file */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(The ``package://`` URI of the file to add.)""";
          } file;
          // Symbol: drake::multibody::parsing::AddModel::name
          struct /* name */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""(The model instance name.)""";
          } name;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("default_free_body_pose", default_free_body_pose.doc),
              std::make_pair("default_joint_positions", default_joint_positions.doc),
              std::make_pair("file", file.doc),
              std::make_pair("name", name.doc),
            };
          }
        } AddModel;
        // Symbol: drake::multibody::parsing::AddModelInstance
        struct /* AddModelInstance */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Directive to add an empty, named model instance to a scene.)""";
          // Symbol: drake::multibody::parsing::AddModelInstance::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::AddModelInstance::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::AddModelInstance::name
          struct /* name */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""(The model instance name.)""";
          } name;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("name", name.doc),
            };
          }
        } AddModelInstance;
        // Symbol: drake::multibody::parsing::AddWeld
        struct /* AddWeld */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Directive to add a weld between two named frames, a parent and a
child.)""";
          // Symbol: drake::multibody::parsing::AddWeld::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::AddWeld::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::AddWeld::X_PC
          struct /* X_PC */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Relative transform between the parent frame P and the child frame C.
If unspecified, the Identity transform will be used.)""";
          } X_PC;
          // Symbol: drake::multibody::parsing::AddWeld::child
          struct /* child */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc =
R"""(Child frame. Can (and should) specify scope.)""";
          } child;
          // Symbol: drake::multibody::parsing::AddWeld::parent
          struct /* parent */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""(Parent frame. Can specify scope.)""";
          } parent;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("X_PC", X_PC.doc),
              std::make_pair("child", child.doc),
              std::make_pair("parent", parent.doc),
            };
          }
        } AddWeld;
        // Symbol: drake::multibody::parsing::FlattenModelDirectives
        struct /* FlattenModelDirectives */ {
          // Source: drake/multibody/parsing/process_model_directives.h
          const char* doc =
R"""(Flatten model directives into a single object.

This function removes all AddDirectives directives from the given
``directives``, locating the file via the given ``package_map``,
parsing it, and updating the names of its items to add any namespace
prefix requested by the ``model_namespace`` of the directive. The
resulting directives are appended to ``out``.

The results of FlattenModelDirectives are semantically identical to
``directives``. FlattenModelDirectives is therefore also idempotent.

This flattening is intended to assist with creating reproducible
simulation scenarios and with hashing; it can also be useful in
debugging.)""";
        } FlattenModelDirectives;
        // Symbol: drake::multibody::parsing::GetScopedFrameByName
        struct /* GetScopedFrameByName */ {
          // Source: drake/multibody/parsing/scoped_names.h
          const char* doc =
R"""(Equivalent to ``GetScopedFrameByNameMaybe``, but throws if the frame
is not found.)""";
        } GetScopedFrameByName;
        // Symbol: drake::multibody::parsing::GetScopedFrameByNameMaybe
        struct /* GetScopedFrameByNameMaybe */ {
          // Source: drake/multibody/parsing/scoped_names.h
          const char* doc =
R"""(Finds an optionally model-scoped frame.

Returns ``nullptr`` if the frame is not found, as well as all the
error cases of ``MultibodyPlant∷HasFrameNamed(std∷string_view)``.)""";
        } GetScopedFrameByNameMaybe;
        // Symbol: drake::multibody::parsing::LoadModelDirectives
        struct /* LoadModelDirectives */ {
          // Source: drake/multibody/parsing/process_model_directives.h
          const char* doc = R"""()""";
        } LoadModelDirectives;
        // Symbol: drake::multibody::parsing::LoadModelDirectivesFromString
        struct /* LoadModelDirectivesFromString */ {
          // Source: drake/multibody/parsing/process_model_directives.h
          const char* doc = R"""()""";
        } LoadModelDirectivesFromString;
        // Symbol: drake::multibody::parsing::ModelDirective
        struct /* ModelDirective */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Union structure for model directives.

Note:
    This was designed before support for ``std∷variant<>`` was around,
    and thus we used a parent field, rather than a YAML tag, to
    designate the intended type for the directive.)""";
          // Symbol: drake::multibody::parsing::ModelDirective::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::ModelDirective::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::ModelDirective::add_collision_filter_group
          struct /* add_collision_filter_group */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } add_collision_filter_group;
          // Symbol: drake::multibody::parsing::ModelDirective::add_directives
          struct /* add_directives */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } add_directives;
          // Symbol: drake::multibody::parsing::ModelDirective::add_frame
          struct /* add_frame */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } add_frame;
          // Symbol: drake::multibody::parsing::ModelDirective::add_model
          struct /* add_model */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } add_model;
          // Symbol: drake::multibody::parsing::ModelDirective::add_model_instance
          struct /* add_model_instance */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } add_model_instance;
          // Symbol: drake::multibody::parsing::ModelDirective::add_weld
          struct /* add_weld */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } add_weld;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("add_collision_filter_group", add_collision_filter_group.doc),
              std::make_pair("add_directives", add_directives.doc),
              std::make_pair("add_frame", add_frame.doc),
              std::make_pair("add_model", add_model.doc),
              std::make_pair("add_model_instance", add_model_instance.doc),
              std::make_pair("add_weld", add_weld.doc),
            };
          }
        } ModelDirective;
        // Symbol: drake::multibody::parsing::ModelDirectives
        struct /* ModelDirectives */ {
          // Source: drake/multibody/parsing/model_directives.h
          const char* doc =
R"""(Top-level structure for a model directives yaml file schema.)""";
          // Symbol: drake::multibody::parsing::ModelDirectives::IsValid
          struct /* IsValid */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } IsValid;
          // Symbol: drake::multibody::parsing::ModelDirectives::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::multibody::parsing::ModelDirectives::directives
          struct /* directives */ {
            // Source: drake/multibody/parsing/model_directives.h
            const char* doc = R"""()""";
          } directives;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("directives", directives.doc),
            };
          }
        } ModelDirectives;
        // Symbol: drake::multibody::parsing::ModelInstanceInfo
        struct /* ModelInstanceInfo */ {
          // Source: drake/multibody/parsing/model_instance_info.h
          const char* doc =
R"""(Convenience structure to hold all of the information to add a model
instance from a file.)""";
          // Symbol: drake::multibody::parsing::ModelInstanceInfo::X_PC
          struct /* X_PC */ {
            // Source: drake/multibody/parsing/model_instance_info.h
            const char* doc = R"""()""";
          } X_PC;
          // Symbol: drake::multibody::parsing::ModelInstanceInfo::child_frame_name
          struct /* child_frame_name */ {
            // Source: drake/multibody/parsing/model_instance_info.h
            const char* doc =
R"""(This is the unscoped frame name belonging to ``model_instance``.)""";
          } child_frame_name;
          // Symbol: drake::multibody::parsing::ModelInstanceInfo::model_instance
          struct /* model_instance */ {
            // Source: drake/multibody/parsing/model_instance_info.h
            const char* doc = R"""()""";
          } model_instance;
          // Symbol: drake::multibody::parsing::ModelInstanceInfo::model_name
          struct /* model_name */ {
            // Source: drake/multibody/parsing/model_instance_info.h
            const char* doc = R"""(Model name (possibly scoped).)""";
          } model_name;
          // Symbol: drake::multibody::parsing::ModelInstanceInfo::model_path
          struct /* model_path */ {
            // Source: drake/multibody/parsing/model_instance_info.h
            const char* doc = R"""(File path.)""";
          } model_path;
          // Symbol: drake::multibody::parsing::ModelInstanceInfo::parent_frame_name
          struct /* parent_frame_name */ {
            // Source: drake/multibody/parsing/model_instance_info.h
            const char* doc =
R"""(WARNING: This is the *unscoped* parent frame, assumed to be unique.)""";
          } parent_frame_name;
        } ModelInstanceInfo;
        // Symbol: drake::multibody::parsing::ProcessModelDirectives
        struct /* ProcessModelDirectives */ {
          // Source: drake/multibody/parsing/process_model_directives.h
          const char* doc_2args =
R"""(Parses the given model directives using the given parser. The
MultibodyPlant (and optionally SceneGraph) being modified are
implicitly associated with the Parser object. Returns the list of
added models.)""";
          // Source: drake/multibody/parsing/process_model_directives.h
          const char* doc_4args =
R"""(Processes model directives for a given MultibodyPlant.)""";
        } ProcessModelDirectives;
        // Symbol: drake::multibody::parsing::ResolveModelDirectiveUri
        struct /* ResolveModelDirectiveUri */ {
          // Source: drake/multibody/parsing/process_model_directives.h
          const char* doc =
R"""(Converts URIs into filesystem absolute paths.

ModelDirectives refer to their resources by URIs like
``package://somepackage/somepath/somefile.sdf``, where somepackage
refers to the ROS-style package.xml system.)""";
        } ResolveModelDirectiveUri;
      } parsing;
    } multibody;
  } drake;
} pydrake_doc_multibody_parsing;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
