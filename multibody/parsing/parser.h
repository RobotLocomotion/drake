#pragma once

#include <string>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

namespace internal {
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
///
/// The output of parsing is one or more model instances added to the
/// MultibodyPlant provided to the parser at construction.
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
/// Drake Model Directives are only available via AddAllModelsFromFile. The
/// single-model methods (AddModelFromFile, AddModelFromString) cannot load
/// model directives.
///
/// For more documentation of Drake-specific treatment of these input formats,
/// see @ref multibody_parsing.
///
/// When parsing literal quantities, %Parser assumes SI units and radians in the
/// absence of units specified by the format itself. This includes the literals
/// in the explicitly specified files as well as referenced files such as OBJ
/// or other data file formats.
class Parser final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Parser)

  /// Creates a Parser that adds models to the given plant and (optionally)
  /// scene_graph.
  ///
  /// @param plant A pointer to a mutable MultibodyPlant object to which parsed
  ///   model(s) will be added; `plant->is_finalized()` must remain `false` for
  ///   as long as the @p plant is in used by `this`.
  /// @param scene_graph A pointer to a mutable SceneGraph object used for
  ///   geometry registration (either to model visual or contact geometry).
  ///   May be nullptr.
  explicit Parser(
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr);

  /// Gets a mutable reference to the plant that will be modified by this
  /// parser.
  MultibodyPlant<double>& plant() { return *plant_; }

  /// Gets a mutable reference to the PackageMap used by this parser.
  PackageMap& package_map() { return package_map_; }

  /// Cause all subsequent Add*Model*() operations to use strict parsing;
  /// warnings will be treated as errors.
  void SetStrictParsing() { is_strict_ = true; }

  /// Parses the input file named in @p file_name and adds all of its model(s)
  /// to @p plant.
  ///
  /// @param file_name The name of the file to be parsed. The file type will be
  /// inferred from the extension.
  /// @returns The set of model instance indices for the newly added models,
  /// including nested models.
  /// @throws std::exception in case of errors.
  std::vector<ModelInstanceIndex> AddAllModelsFromFile(
      const std::string& file_name);

  /// Parses the input file named in @p file_name and adds one top-level model
  /// to @p plant. It is an error to call this using any file that adds more
  /// than one model instance.
  ///
  /// @note This function might create additional model instances corresponding
  /// to nested models found in the top level file. This means that elements
  /// contained by the returned model instance may not comprise all of the
  /// added elements due to how model instances are mutually exclusive and not
  /// hierarchical (#14043).
  ///
  /// @sa http://sdformat.org/tutorials?tut=composition&ver=1.7 for details on
  /// nesting in SDFormat.
  ModelInstanceIndex AddModelFromFile(
      const std::string& file_name,
      const std::string& model_name = {});

  /// Provides same functionality as AddModelFromFile, but instead parses the
  /// model description text data via @p file_contents with format dictated by
  /// @p file_type.
  ///
  /// @param file_contents The model data to be parsed.
  /// @param file_type The data format; must be one of the filename suffixes
  /// listed above, *without* the leading dot (.) .
  /// @param model_name The name given to the newly created instance of this
  /// model. If empty, the model name provided by the input text will be used.
  /// @returns The instance index for the newly added model.
  /// @throws std::exception in case of errors.
  ModelInstanceIndex AddModelFromString(
      const std::string& file_contents,
      const std::string& file_type,
      const std::string& model_name = {});

 private:
  friend class internal::CompositeParse;

  bool is_strict_{false};
  PackageMap package_map_;
  drake::internal::DiagnosticPolicy diagnostic_policy_;
  MultibodyPlant<double>* const plant_;
};

}  // namespace multibody
}  // namespace drake

