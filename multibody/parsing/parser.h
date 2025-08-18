#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/collision_filter_groups.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

namespace internal {
class CollisionFilterGroupResolver;
class CompositeParse;
}  // namespace internal

/// Parses model description input into a MultibodyPlant and (optionally) a
/// SceneGraph. A variety of input formats are supported, and are recognized by
/// filename suffixes:
///
/// File format              | Filename suffix
/// ------------------------ | ---------------
/// URDF                     | ".urdf"
/// SDFormat                 | ".sdf"
/// MJCF (Mujoco XML)        | ".xml"
/// Drake Model Directives   | ".dmd.yaml"
/// Wavefront OBJ            | ".obj"
///
/// The output of parsing is one or more model instances added to the
/// MultibodyPlant provided to the parser at construction.
///
/// For an introductory tutorial about parsing, see the <a
/// href="https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/authoringmultibodysimulation-8159e3be6dc048a6a8ab6876017def4b">Authoring
/// a Multibody Simulation</a> page.
///
/// SDFormat files may contain multiple `<model>` elements.  New model
/// instances will be added to @p plant for each `<model>` tag in the file.
///
/// @note Adding multiple root-level models, i.e, `<model>`s directly under
/// `<sdf>`, is deprecated. If you need multiple models in a single file,
/// please use an SDFormat world tag.
///
/// URDF files contain a single `<robot>` element.  Only a single model
/// instance will be added to @p plant.
///
/// MJCF (MuJoCo XML) files typically contain many bodies, they will all be
/// added as a single model instance in the @p plant.
///
/// Drake Model Directives are only available via AddModels or
/// AddModelsFromString. The single-model methods (AddModelFromFile,
/// AddModelFromString) cannot load model directives.
///
/// OBJ files will infer a model with a single body from the geometry. The OBJ
/// file must contain a _single_ object (in the OBJ-file sense). The body's mass
/// properties are computed based on uniform distribution of material in the
/// enclosed volume of the mesh (with the approximate density of water: 1000
/// kg/m³). If the mesh is not a closed manifold, this can produce unexpected
/// results. The spatial inertia of the body is measured at the body frame's
/// origin. The body's frame is coincident and fixed with the frame the mesh's
/// vertices are measured and expressed in. The mesh's vertices are assumed to
/// be measured in units of _meters_.
///
/// The name of the model and body are determined according to the following
/// prioritized protocol:
///
///   - The non-empty `model_name`, if given (e.g., in AddModelFromFile()).
///   - If the object is named in the obj file, that object name is used.
///   - Otherwise, the base name of the file name is used (i.e., the file name
///     with the prefixed directory and extension removed).
///
/// If the underlying plant is registered with a SceneGraph instance, the mesh
/// will also be used for all three roles: illustration, perception, and
/// proximity.
///
/// @warning AddModelsFromString() cannot be passed OBJ file contents yet.
///
/// For more documentation of Drake-specific treatment of these input formats,
/// see @ref multibody_parsing.
///
/// When parsing literal quantities, %Parser assumes SI units and radians in the
/// absence of units specified by the format itself. This includes the literals
/// in the explicitly specified files as well as referenced files such as OBJ
/// or other data file formats.
///
/// MultibodyPlant requires that model instances have unique names. To support
/// loading multiple instances of the same model file(s) into a plant, Parser
/// offers a few different strategies.
///
/// Parser has constructors that take a model name prefix, which gets applied to
/// all models loaded with that Parser instance. The resulting workflow makes
/// multiple parsers to build models for a single plant:
/// @code
///  Parser left_parser(plant, "left");
///  Parser right_parser(plant, "right");
///  left_parser.AddModels(arm_model);  // "left::arm"
///  right_parser.AddModels(arm_model);  // "right::arm"
///  left_parser.AddModels(gripper_model);  // "left::gripper"
///  right_parser.AddModels(gripper_model);  // "right::gripper"
/// @endcode
///
/// For situations where it is convenient to load a model many times, Parser
/// offers optional auto-renaming. When auto-renaming is enabled, name
/// collisions will be resolved by adding a subscript to the name.
/// @code
///  Parser parser(plant);
///  parser.SetAutoRenaming(true);
///  // Subscripts are compact, and start at 1.
///  parser.AddModels(rock);  // "rock"
///  parser.AddModels(rock);  // "rock_1"
///  parser.AddModels(rock);  // "rock_2"
///  // Subscripts of different base names are independent.
///  parser.AddModels(stone);  // "stone"
///  parser.AddModels(stone);  // "stone_1"
///  parser.AddModels(stone);  // "stone_2"
/// @endcode
///
/// (Advanced) In the rare case where the user is parsing into a MultibodyPlant
/// and SceneGraph but has created them one at a time instead of using the more
/// convenient AddMultibodyPlant() or AddMultibodyPlantSceneGraph() functions,
/// the Parser constructors accept an optional SceneGraph pointer to specify
/// which SceneGraph to parse into. If it is provided and non-null and the
/// MultibodyPlant is not registered as a source, the Parser will perform the
/// SceneGraph registration into the given plant. We describe this option only
/// for completeness; we strongly discourage anyone from taking advantage of
/// this feature.
class Parser final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Parser);

  /// Create a Parser given a DiagramBuilder (and optionally plant and
  /// scene_graph). If specified, the resulting parser will apply
  /// `model_name_prefix` to the names of any models parsed. `builder` may be
  /// nullptr, but then you must specify a `plant`.
  ///
  /// @pre Either the given `builder` contains a MultibodyPlant system named
  ///   "plant" or else the provided `plant` is non-null.
  /// @pre If both `builder` and `plant` are specified, then
  ///   plant ∈ builder.GetSystems()
  /// @param scene_graph A pointer to a mutable SceneGraph object used for
  ///   geometry registration (either to model visual or contact geometry).
  ///   May be nullptr.
  /// @pre If both `builder` and `scene_graph` are specified, then
  ///   scene_graph ∈ builder.GetSystems().
  explicit Parser(systems::DiagramBuilder<double>* builder,
                  MultibodyPlant<double>* plant = nullptr,
                  geometry::SceneGraph<double>* scene_graph = nullptr,
                  std::string_view model_name_prefix = {});

  /// Creates a Parser that adds models to the given plant and (optionally)
  /// scene_graph.
  ///
  /// @param plant A pointer to a mutable MultibodyPlant object to which parsed
  ///   model(s) will be added; `plant->is_finalized()` must remain `false` for
  ///   as long as the @p plant is in use by `this`.
  /// @param scene_graph A pointer to a mutable SceneGraph object used for
  ///   geometry registration (either to model visual or contact geometry).
  ///   May be nullptr.
  explicit Parser(MultibodyPlant<double>* plant,
                  geometry::SceneGraph<double>* scene_graph = nullptr);

  /// Creates a Parser that adds models to the given plant and scene_graph. The
  /// resulting parser will apply `model_name_prefix` to the names of any
  /// models parsed.
  ///
  /// @param plant A pointer to a mutable MultibodyPlant object to which parsed
  ///   model(s) will be added; `plant->is_finalized()` must remain `false` for
  ///   as long as the @p plant is in use by `this`.
  /// @param scene_graph A pointer to a mutable SceneGraph object used for
  ///   geometry registration (either to model visual or contact geometry).
  ///   May be nullptr.
  /// @param model_name_prefix A string that will be added as a scoped name
  ///   prefix to the names of any models loaded by this parser;
  ///   when empty, no scoping will be added.
  Parser(MultibodyPlant<double>* plant,
         geometry::SceneGraph<double>* scene_graph,
         std::string_view model_name_prefix);

  /// Creates a Parser that adds models to the given plant and scene_graph. The
  /// resulting parser will apply `model_name_prefix` to the names of any
  /// models parsed.
  ///
  /// @param plant A pointer to a mutable MultibodyPlant object to which parsed
  ///   model(s) will be added; `plant->is_finalized()` must remain `false` for
  ///   as long as the @p plant is in use by `this`.
  /// @param model_name_prefix A string that will be added as a scoped name
  ///   prefix to the names of any models loaded by this parser;
  ///   when empty, no scoping will be added.
  Parser(MultibodyPlant<double>* plant, std::string_view model_name_prefix);

  ~Parser();

  /// Gets a mutable pointer to the DiagramBuilder that will be modified by
  /// this parser, or nullptr if this parser does not have a DiagramBuilder.
  systems::DiagramBuilder<double>* builder() { return builder_; }

  /// Gets a mutable reference to the plant that will be modified by this
  /// parser.
  MultibodyPlant<double>& plant() { return *plant_; }

  /// Gets a mutable pointer to the SceneGraph that will be modified by this
  /// parser, or nullptr if this parser does not have a SceneGraph.
  geometry::SceneGraph<double>* scene_graph();

  /// Gets a mutable reference to the PackageMap used by this parser.
  PackageMap& package_map() { return package_map_; }

  /// Cause all subsequent Add*Model*() operations to use strict parsing;
  /// warnings will be treated as errors.
  void SetStrictParsing() { is_strict_ = true; }

  /// Enable or disable auto-renaming. It is disabled by default.
  /// @see the Parser class documentation for more detail.
  void SetAutoRenaming(bool value) { enable_auto_rename_ = value; }

  /// Get the current state of auto-renaming.
  /// @see the Parser class documentation for more detail.
  bool GetAutoRenaming() const { return enable_auto_rename_; }

  /// Gets the accumulated set of collision filter definitions seen by this
  /// parser.
  ///
  /// There are two kinds of names in the returned data: group names and body
  /// names. Both may occur within scoped names indicating the model instance
  /// where they are defined. Note that the model instance names used in the
  /// returned data will reflect the current names in plant() at the time this
  /// accessor is called (see MultibodyPlant::RenameModelInstance()), but the
  /// local group and body names will be the names seen during parsing.
  CollisionFilterGroups GetCollisionFilterGroups() const;

  /// Parses the input file named in @p file_name and adds all of its model(s)
  /// to @p plant.
  ///
  /// @param file_name The name of the file to be parsed. The file type will be
  /// inferred from the extension.
  /// @returns The set of model instance indices for the newly added models,
  /// including nested models.
  /// @throws std::exception in case of errors.
  std::vector<ModelInstanceIndex> AddModels(
      const std::filesystem::path& file_name);

  /// Parses the input file named in @p url and adds all of its model(s) to
  /// @p plant. The allowed URL schemes are either `file://` for local files
  /// or `package://` (or `model://`) to use this Parser's `package_map()`.
  ///
  /// @param url The file to be parsed. The file type will be inferred from
  /// the extension.
  /// @returns The set of model instance indices for the newly added models,
  /// including nested models.
  /// @throws std::exception in case of errors.
  std::vector<ModelInstanceIndex> AddModelsFromUrl(const std::string& url);

  /// Provides same functionality as AddModels, but instead parses
  /// the model description text data via @p file_contents with format dictated
  /// by @p file_type.
  ///
  /// @param file_contents The model data to be parsed.
  /// @param file_type The data format; must be one of the filename suffixes
  /// listed above, *without* the leading dot (.).
  /// @returns The set of model instance indices for the newly added models,
  /// including nested models.
  /// @throws std::exception in case of errors.
  std::vector<ModelInstanceIndex> AddModelsFromString(
      const std::string& file_contents, const std::string& file_type);

 private:
  friend class internal::CompositeParse;

  // This is called back from CompositeParse::Finish().
  void ResolveCollisionFilterGroupsFromCompositeParse(
      internal::CollisionFilterGroupResolver* resolver);

  bool is_strict_{false};
  bool enable_auto_rename_{false};
  PackageMap package_map_;
  drake::internal::DiagnosticPolicy diagnostic_policy_;
  systems::DiagramBuilder<double>* const builder_;
  MultibodyPlant<double>* plant_;
  std::optional<std::string> model_name_prefix_;
  struct ParserInternalData;
  std::unique_ptr<ParserInternalData> data_;
};

}  // namespace multibody
}  // namespace drake
