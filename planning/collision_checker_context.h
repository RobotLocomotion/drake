#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {

/** This class represents the data necessary for CollisionChecker to operate
 safely across multiple threads in its `const` API. Instances of this class
 are owned and managed by a particular CollisionChecker.

 If using OMP to perform parallel const queries on a CollisionChecker, it will
 never be necessary to interact with %CollisionCheckerContext instances. Only if
 using some other threading paradigm will it be necessary to work with "stand
 alone" instances. See CollisionChecker's documentation for more details.

 In all cases, modifying context should happen through
 CollisionChecker::PerformOperationAgainstAllModelContexts(). Modifying the
 contained Drake Contexts directly is generally erroneous.

 @ingroup planning_collision_checker */
class CollisionCheckerContext {
 public:
  /** \name Does not allow copy, move, or assignment generally.

   Protected copy construction is enabled for sub-classes to use in their
   implementation of DoClone(). */
  //@{
  CollisionCheckerContext& operator=(const CollisionCheckerContext&) = delete;
  CollisionCheckerContext(CollisionCheckerContext&&) = delete;
  CollisionCheckerContext& operator=(CollisionCheckerContext&&) = delete;
  //@}

  /** The resulting object stores an alias to `model`; the passed model should
   have a lifetime greater than the constructed object.
   @pre model is not null. */
  explicit CollisionCheckerContext(const RobotDiagram<double>* model);

  virtual ~CollisionCheckerContext();

  std::unique_ptr<CollisionCheckerContext> Clone() const { return DoClone(); }

  /** Gets the contained model context. */
  const systems::Context<double>& model_context() const {
    return *model_context_;
  }

  /** Gets the contained plant context. */
  const systems::Context<double>& plant_context() const {
    return *plant_context_;
  }

  /** Gets the contained scene graph context. */
  const systems::Context<double>& scene_graph_context() const {
    return *scene_graph_context_;
  }

  /** Gets the scene graph geometry query object. */
  const geometry::QueryObject<double>& GetQueryObject() const;

  // TODO(SeanCurtis-TRI) Eliminate these public members that provide access to
  // mutable sub-contexts.

  /* (Internal use only) Gets the contained model context. */
  systems::Context<double>& mutable_model_context() { return *model_context_; }

  /* (Internal use only) Gets the contained plant context. */
  systems::Context<double>& mutable_plant_context() { return *plant_context_; }

  /* (Internal use only) Gets the contained scene graph context. */
  systems::Context<double>& mutable_scene_graph_context() {
    return *scene_graph_context_;
  }

 protected:
  /** Derived classes can use this copy constructor to help implement their own
   DoClone() methods. */
  CollisionCheckerContext(const CollisionCheckerContext& other);

  /** Allow derived context types to implement additional clone behavior. */
  virtual std::unique_ptr<CollisionCheckerContext> DoClone() const;

 private:
  /* The resulting object stores an alias to `model`; the passed model should
   have a lifetime greater than the constructed object.

   @pre `model` is not null.
   @pre `model_context` is not null and is allocated by `model`. */
  CollisionCheckerContext(
      const RobotDiagram<double>* model,
      std::unique_ptr<systems::Context<double>> model_context);

  const RobotDiagram<double>& model_;

  std::unique_ptr<systems::Context<double>> model_context_;
  /* These are aliases into model_context_. */
  systems::Context<double>* const plant_context_;
  systems::Context<double>* const scene_graph_context_;
};

}  // namespace planning
}  // namespace drake
