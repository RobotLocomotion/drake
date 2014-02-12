function obj = constructMultipleTimeLinearPostureConstraint(type,use_mex,varargin)
use_mex = logical(use_mex);
if(use_mex)
  if(isa(varargin{1},'RigidBodyManipulator')||isa(varargin{1},'TimeSteppingRigidBodyManipulator'))
    varargin{1} = varargin{1}.getMexModelPtr;
  end
end
switch type
  case MultipleTimeLinearPostureConstraint.PostureChangeConstraint
    if(use_mex)
      obj = constructPtrPostureChangeConstraintmex(varargin{:});
    else
      obj = PostureChangeConstraint(varargin{:});
    end
end
end