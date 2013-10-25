function obj = constructSingleTimeKinematicConstraint(type,use_mex,varargin)
use_mex = logical(use_mex);
if(use_mex)
  if(isa(varargin{1},'RigidBodyManipulator')||isa(varargin{1},'TimeSteppingRigidBodyManipulator'))
    varargin{1} = varargin{1}.getMexModelPtr;
  end
end
switch type
  case SingleTimeKinematicConstraint.WorldCoMConstraint
    if(use_mex)
      obj = constructPtrWorldCoMConstraintmex(varargin{:});
    else
      obj = WorldCoMConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldPositionConstraint
    if(use_mex)
      obj = constructPtrWorldPositionConstraintmex(varargin{:});
    else
      obj = WorldPositionConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldQuatConstraint
    if(use_mex)
      obj = constructPtrWorldQuatConstraintmex(varargin{:});
    else
      obj = WorldQuatConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldEulerConstraint
    if(use_mex)
      obj = constructPtrWorldEulerConstraintmex(varargin{:});
    else
      obj = WorldEulerConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldGazeOrientConstraint
    if(use_mex)
      obj = constructPtrWorldGazeOrientConstraintmex(varargin{:});
    else
      obj = WorldGazeOrientConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldGazeDirConstraint
    if(use_mex)
      obj = constructPtrWorldGazeDirConstraintmex(varargin{:});
    else
      obj = WorldGazeDirConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldGazeTargetConstraint
    if(use_mex)
      obj = constructPtrWorldGazeTargetConstraintmex(varargin{:});
    else
      obj = WorldGazeTargetConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.AllBodiesClosestDistanceConstraint
    if(use_mex)
      obj = constructPtrAllBodiesClosestDistanceConstraintmex(varargin{:});
    else
      obj = AllBodiesClosestDistanceConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.Point2PointDistanceConstraint
    if(use_mex)
      obj = constructPtrPoint2PointDistanceConstraintmex(varargin{:});
    else
      obj = Point2PointDistanceConstraint(varargin{:});
    end
  case SingleTimeKinematicConstraint.WorldPositionInFrameConstraint
    if(use_mex)
      obj = constructPtrWorldPositionInFrameConstraintmex(varargin{:});
    else
      obj = WorldPositionInFrameConstraint(varargin{:});
    end
  otherwise
    error('constructSingleTimeKinematicConstraint:bad_type', ...
          'The specified constraint type is not supported by this function.');
end
end
