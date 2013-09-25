function obj = constructMultipleTimeKinematicConstraint(type,use_mex,varargin)
use_mex = logical(use_mex);
if(use_mex)
  if(isa(varargin{1},'RigidBodyManipulator')||isa(varargin{1},'TimeSteppingRigidBodyManipulator'))
    varargin{1} = varargin{1}.getMexModelPtr;
  end
end
switch type
  case MultipleTimeKinematicConstraint.WorldFixedPositionConstraint
    if(use_mex)
      obj = constructPtrWorldFixedPositionConstraintmex(varargin{:});
    else
      obj = WorldFixedPositionConstraint(varargin{:});
    end
  case MultipleTimeKinematicConstraint.WorldFixedOrientConstraint
    if(use_mex)
      obj = constructPtrWorldFixedOrientConstraintmex(varargin{:});
    else
      obj = WorldFixedOrientConstraint(varargin{:});
    end
  case MultipleTimeKinematicConstraint.WorldFixedBodyPostureConstraint
    if(use_mex)
      obj = constructPtrWorldFixedBodyPostureConstraintmex(varargin{:});
    else
      obj = WorldFixedBodyPostureConstraint(varargin{:});
    end
end
end