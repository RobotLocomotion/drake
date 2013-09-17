function obj = constructKinematicConstraint(type,use_mex,varargin)
use_mex = logical(use_mex);
if(use_mex)
  if(isa(varargin{1},'RigidBodyManipulator')||isa(varargin{1},'TimeSteppingRigidBodyManipulator'))
    varargin{1} = varargin{1}.getMexModelPtr;
  end
end
switch type
  case KinematicConstraint.WorldCoMConstraint
    if(use_mex)
      obj = constructPtrWorldCoMConstraintmex(varargin{:});
    else
      obj = WorldCoMConstraint(varargin{:});
    end
  case KinematicConstraint.WorldPositionConstraint
    if(use_mex)
      obj = constructPtrWorldPositionConstraintmex(varargin{:});
    else
      obj = WorldPositionConstraint(varargin{:});
    end
  case KinematicConstraint.WorldQuatConstraint
    if(use_mex)
      obj = constructPtrWorldQuatConstraintmex(varargin{:});
    else
      obj = WorldQuatConstraint(varargin{:});
    end
  case KinematicConstraint.WorldEulerConstraint
    if(use_mex)
      obj = constructPtrWorldEulerConstraintmex(varargin{:});
    else
      obj = WorldEulerConstraint(varargin{:});
    end
  case KinematicConstraint.WorldGazeOrientConstraint
    if(use_mex)
      obj = constructPtrWorldGazeOrientConstraintmex(varargin{:});
    else
      obj = WorldGazeOrientConstraint(varargin{:});
    end
  case KinematicConstraint.WorldGazeDirConstraint
    if(use_mex)
      obj = constructPtrWorldGazeDirConstraintmex(varargin{:});
    else
      obj = WorldGazeDirConstraint(varargin{:});
    end
  case KinematicConstraint.WorldGazeTargetConstraint
    if(use_mex)
      obj = constructPtrWorldGazeTargetConstraintmex(varargin{:});
    else
      obj = WorldGazeTargetConstraint(varargin{:});
    end
end
end