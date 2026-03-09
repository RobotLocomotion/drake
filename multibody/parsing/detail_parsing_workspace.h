#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

// This forward declaration is necessary to support the definition cycle among
// ParsingWorkspace, ParserInterface, and ParserSelector.
struct ParsingWorkspace;

// ParserInterface is a common interface for format-specific parsers. It
// enables the definition of an abstract parser-selector functor type.
class ParserInterface {
 public:
  virtual ~ParserInterface();

  // Parses a model from the input file specified by @p data_source and adds it
  // to @p plant. A new model instance will be added to @p plant.
  //
  // @param data_source
  //   The model data to be parsed.
  // @param model_name
  //   The name given to the newly created instance of this model.  If
  //   empty, the model name found within the model data will be used.
  // @param parent_model_name Optional name of parent model. If set, the model
  //   name of the parsed model (either `model_name` or from the "name"
  //   attribute) will be prefixed with the parent_model_name, using the
  //   SDFormat scope delimiter "::". The prefixed name will used as the name
  //   given to the newly created instance of this model.
  // @param workspace
  //   The ParsingWorkspace.
  // @returns The model instance index for the newly added model, or
  //          std::nullopt if no model instance was allocated. An instance will
  //          be allocated as long as a valid model name can be constructed, by
  //          consulting the supplied name parameters, and the model name (if
  //          any) found in the @p data_source.
  virtual std::optional<ModelInstanceIndex> AddModel(
      const DataSource& data_source, const std::string& model_name,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) = 0;

  // Parses a model from the input file specified by @p data_source adds all of
  // its contents into an existing model instance specified by @p
  // model_instance. @p model_instance must already exist in @p plant.
  //
  // @param data_source
  //   The model data to be parsed.
  // @param model_name
  //   The name given to the newly created instance of this model.  If
  //   empty, the model name found within the model data will be used.
  // @param merge_into_model_instance
  //   The model instance into which the contents of the model will be added.
  // @param workspace
  //   The ParsingWorkspace.
  // @returns The name of the the model that was loaded from @p data_source.
  //   Note, this is not the name of the @p model_instance. Instead, it's the
  //   name of the original model that has now been merged into @p
  //   model_instance.
  virtual std::string MergeModel(
      const DataSource& /* data_source */, const std::string& /* model_name */,
      ModelInstanceIndex /* merge_into_model_instance*/,
      const ParsingWorkspace& /* workspace */) {
    throw std::runtime_error(
        fmt::format("MergeModel is not implemented for this input type"));
  }

  // Parses all models from the input file specified by @p data_source and adds
  // them to @p plant. New model instances will be added to @p plant.
  //
  // @param data_source
  //   The model data to be parsed.
  // @param parent_model_name Optional name of parent model. If set, the model
  //   names of all parsed models will be prefixed with the parent_model_name,
  //   using the SDFormat scope delimiter "::". The prefixed name will used as
  //   the name given to the newly created instances of these models.
  // @param workspace
  //   The ParsingWorkspace.
  // @returns The model instance indices for the newly added models, or an
  //          empty vector if no model instances were allocated. Instances will
  //          be allocated as long as valid model names can be constructed, by
  //          consulting the model names (if any) found in the @p data_source.
  virtual std::vector<ModelInstanceIndex> AddAllModels(
      const DataSource& data_source,
      const std::optional<std::string>& parent_model_name,
      const ParsingWorkspace& workspace) = 0;
};

// The function type of a parser-selector. This abstraction helps avoid
// dependencies between format-specific parsers. Only concrete ParserSelector
// implementations should induce dependencies on format-specific parsers.
//
// Note that the behavior when the format is unknown is left unspecified
// here. Functors for use in production may return a do-nothing interface
// object to allow further best-effort parsing; functors used for test programs
// may assert or throw.
//
// @param policy
//   The DiagnosticPolicy object, for processing errors and warnings. It is
//   only used to report selection errors. It is not aliased or retained.
// @param filename
//   The name of the input file to parse; it is only used to detect the likely
//   format of the data.
// @returns A reference to a ParserInterface.
using ParserSelector = std::function<ParserInterface&(
    const drake::internal::DiagnosticPolicy& policy,
    const std::string& filename)>;

struct ParsingOptions {
  bool enable_auto_renaming{false};
};

// ParsingWorkspace bundles the commonly-needed elements for parsing routines.
// It owns nothing; all members are references or pointers to objects owned
// elsewhere.
//
// Note that code using this struct may pass it via const-ref, but the
// indicated plant and collision resolver objects will still be mutable; only
// the pointer values of the plant and resolver within the struct are const.
struct ParsingWorkspace {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParsingWorkspace);

  // All parameters are aliased; they must have a lifetime greater than that of
  // this struct.
  ParsingWorkspace(
      const ParsingOptions& options_in, const PackageMap& package_map_in,
      const drake::internal::DiagnosticPolicy& diagnostic_in,
      systems::DiagramBuilder<double>* builder_in,
      MultibodyPlant<double>* plant_in,
      internal::CollisionFilterGroupResolver* collision_resolver_in,
      ParserSelector parser_selector_in)
      : options(options_in),
        package_map(package_map_in),
        diagnostic(diagnostic_in),
        builder(builder_in),
        plant(plant_in),
        scene_graph(plant_in->geometry_source_is_registered()
                        ? plant_in->GetMutableSceneGraphPreFinalize()
                        : nullptr),
        collision_resolver(collision_resolver_in),
        parser_selector(parser_selector_in) {
    DRAKE_DEMAND(plant != nullptr);
    DRAKE_DEMAND(collision_resolver != nullptr);
    DRAKE_DEMAND(parser_selector != nullptr);
  }

  const ParsingOptions& options;
  const PackageMap& package_map;
  const drake::internal::DiagnosticPolicy& diagnostic;
  systems::DiagramBuilder<double>* const builder;
  MultibodyPlant<double>* const plant;
  geometry::SceneGraph<double>* const scene_graph;
  internal::CollisionFilterGroupResolver* const collision_resolver;
  const ParserSelector parser_selector;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
