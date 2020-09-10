#pragma once
/// @file
/// Provides directives for building scenes.
/// See `multibody/parsing/README_model_directives.md` for more info.

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/common/schema/transform.h"
#include "drake/common/text_logging.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Directive to add a weld between two named frames, a parent and a child.
struct AddWeld {
  bool IsValid() const {
    if (parent.empty()) {
      drake::log()->error("add_weld: `parent` must be non-empty");
      return false;
    } else if (child.empty()) {
      drake::log()->error("add_weld: `child` must be non-empty");
      return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(parent));
    a->Visit(DRAKE_NVP(child));
  }

  // Parent frame. Can specify scope.
  std::string parent;
  // Child frame. Can (and should) specify scope.
  std::string child;
};

/// Directive to add a model from a urdf or sdf file to a scene, using a given
/// name for the added instance.
struct AddModel {
  bool IsValid() const {
    if (file.empty()) {
      drake::log()->error("add_model: `file` must be non-empty");
      return false;
    } else if (name.empty()) {
      drake::log()->error("add_model: `name` must be non-empty");
      return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(file));
    a->Visit(DRAKE_NVP(name));
  }

  std::string file;
  // Model instance name.
  std::string name;
};

/// Directive to add an empty, named model instance to a scene.
struct AddModelInstance {
  bool IsValid() const {
    if (name.empty()) {
      drake::log()->error("add_model_instance: `name` must be non-empty");
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
  }

  std::string name;
};

/// Directive to add a path in the filesystem to the resolution of
/// `package://` URIs during the processing of model directives and the
/// models they load.
/// NOTE:  This is expected to be deprecated soon.
struct AddPackagePath {
  bool IsValid() const {
    if (name.empty()) {
      drake::log()->error("add_package_path: `name` must be non-empty.");
      return false;
    } else if (path.empty()) {
      drake::log()->error("add_package_path: `path` must be non-empty.");
      return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(path));
  }

  std::string name;
  std::string path;
};

/// Adds a Frame to the scene.  The added frame must have a name and a
/// transform with a base frame and offset.
struct AddFrame {
  bool IsValid() const {
    if (name.empty()) {
      drake::log()->error("add_frame: `name` must be non-empty");
      return false;
    } else if (!X_PF.base_frame || X_PF.base_frame->empty()) {
      drake::log()->error("add_frame: `X_PF.base_frame` must be defined");
      return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(X_PF));
  }

  // Name of frame to be added. If scope is specified, will override model
  // instance; otherwise, will use `X_PF.base_frame`s instance.
  std::string name;
  // Pose of frame to be added, `F`, w.r.t. parent frame `P` (as defined by
  // `X_PF.base_frame`).
  drake::schema::Transform X_PF;
};

/// Directive to incorporate another model directives file, optionally with
/// its elements prefixed with a namespace.
struct AddDirectives {
  bool IsValid() const {
    if (file.empty()) {
      drake::log()->error("add_directives: `file` must be non-empty");
      return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(file));
    a->Visit(DRAKE_NVP(model_namespace));
  }

  std::string file;
  // Namespaces base model instance for processing directive files.
  // Affects scoping (i.e. the following members):
  // - AddModel::name
  // - AddModelInstance::name
  // - AddFrame::name
  // - AddWeld::parent
  // - AddWeld::child
  // - AddFrame::X_PF::base_frame
  // - AddDirectives::model_namespace
  // See `README.md` for example references and namespacing.
  std::optional<std::string> model_namespace;
};

/// Union structure for top-level model directives yaml file elements.
struct ModelDirective {
  bool IsValid() const {
    const bool unique =
        (add_model.has_value() + add_model_instance.has_value() +
         add_frame.has_value() + add_weld.has_value() +
         add_package_path.has_value() + add_directives.has_value()) == 1;
    if (!unique) {
      drake::log()->error(
          "directive: Specify one of `add_model`, `add_model_instance`, "
          "`add_frame`, `add_package_path`, or `add_directives`");
      return false;
    } else if (add_model) {
      return add_model->IsValid();
    } else if (add_model_instance) {
      return add_model_instance->IsValid();
    } else if (add_frame) {
      return add_frame->IsValid();
    } else if (add_weld) {
      return add_weld->IsValid();
    } else if (add_package_path) {
      return add_package_path->IsValid();
    } else {
      return add_directives->IsValid();
    }
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(add_model));
    a->Visit(DRAKE_NVP(add_model_instance));
    a->Visit(DRAKE_NVP(add_frame));
    a->Visit(DRAKE_NVP(add_weld));
    a->Visit(DRAKE_NVP(add_package_path));
    a->Visit(DRAKE_NVP(add_directives));
  }

  std::optional<AddModel> add_model;
  std::optional<AddModelInstance> add_model_instance;
  std::optional<AddFrame> add_frame;
  std::optional<AddWeld> add_weld;
  std::optional<AddPackagePath> add_package_path;
  std::optional<AddDirectives> add_directives;
};

/// Struct for the model directives yaml file schema.
struct ModelDirectives {
  bool IsValid() const {
    for (auto& directive : directives) {
      if (!directive.IsValid()) return false;
    }
    return true;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(directives));
  }

  std::vector<ModelDirective> directives;
};

/// Syntactic sugar for a common idiom: Construct an add_package_path
/// directive and insert it at the beginning of a ModelDirectives.
inline void AddPackageToModelDirectives(const std::string& package_name,
                                        const std::string& package_path,
                                        ModelDirectives* directives) {
  AddPackagePath add_package_path;
  add_package_path.name = package_name;
  add_package_path.path = package_path;
  ModelDirective directive;
  directive.add_package_path = add_package_path;
  directives->directives.insert(
      directives->directives.begin(), directive);
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
