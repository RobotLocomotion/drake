function obj = constructRigidBodyConstraint(type,use_mex,varargin)
% @param type     -- The type of the non-abstract RigidBodyConstraint. See
%                    RigidBodyConstraint for more details
% @param use_mex  -- A boolean variable. True if only C++ object is constructed. False if
%                    the user wants to construct both MATLAB and C++ objects
sizecheck(use_mex,[1,1]);
use_mex = logical(use_mex);
if(use_mex)
  if(isa(varargin{1},'RigidBodyManipulator')||isa(varargin{1},'TimeSteppingRigidBodyManipulator'))
    varargin{1} = varargin{1}.getMexModelPtr;
  end
end
if(use_mex)
  obj = constructPtrRigidBodyConstraintmex(type,varargin{:});
else
  switch type
    case RigidBodyConstraint.QuasiStaticConstraintType
      obj = QuasiStaticConstraint(varargin{:});
    case RigidBodyConstraint.PostureConstraintType
      obj = PostureConstraint(varargin{:});
    case RigidBodyConstraint.SingleTimeLinearPostureConstraintType
      obj = SingleTimeLinearPostureConstraint(varargin{:});
    case RigidBodyConstraint.AllBodiesClosestDistanceConstraintType
      obj = AllBodiesClosestDistanceConstraint(varargin{:});
    case RigidBodyConstraint.WorldEulerConstraintType
      obj = WorldEulerConstraint(varargin{:});
    case RigidBodyConstraint.WorldGazeDirConstraintType
      obj = WorldGazeDirConstraint(varargin{:});
    case RigidBodyConstraint.WorldGazeOrientConstraintType
      obj = WorldGazeOrientConstraint(varargin{:});
    case RigidBodyConstraint.WorldGazeTargetConstraintType
      obj = WorldGazeTargetConstraint(varargin{:});
    case RigidBodyConstraint.RelativeGazeTargetConstraintType
      obj = RelativeGazeTargetConstraint(varargin{:});
    case RigidBodyConstraint.RelativeGazeDirConstraintType
      obj = RelativeGazeDirConstraint(varargin{:});
    case RigidBodyConstraint.WorldCoMConstraintType
      obj = WorldCoMConstraint(varargin{:});
    case RigidBodyConstraint.WorldPositionConstraintType
      obj = WorldPositionConstraint(varargin{:});
    case RigidBodyConstraint.WorldPositionInFrameConstraintType
      obj = WorldPositionInFrameConstraint(varargin{:});
    case RigidBodyConstraint.WorldQuatConstraintType
      obj = WorldQuatConstraint(varargin{:});
    case RigidBodyConstraint.Point2PointDistanceConstraintType
      obj = Point2PointDistanceConstraint(varargin{:});
    case RigidBodyConstraint.Point2LineSegDistConstraintType
      obj = Point2LineSegDistConstraint(varargin{:});
    case RigidBodyConstraint.WorldFixedPositionConstraintType
      obj = WorldFixedPositionConstraint(varargin{:});
    case RigidBodyConstraint.WorldFixedOrientConstraintType
      obj = WorldFixedOrientConstraint(varargin{:});
    case RigidBodyConstraint.WorldFixedBodyPoseConstraintType
      obj = WorldFixedBodyPoseConstraint(varargin{:});
    case RigidBodyConstraint.PostureChangeConstraintType
      obj = PostureChangeConstraint(varargin{:});
    case RigidBodyConstraint.RelativePositionConstraintType
      obj = RelativePositionConstraint(varargin{:});
    case RigidBodyConstraint.RelativeQuatConstraintType
      obj = RelativeQuatConstraint(varargin{:});
    case RigidBodyConstraint.MinDistanceConstraintType
      obj = MinDistanceConstraint(varargin{:});
    case RigidBodyConstraint.GravityCompensationTorqueConstraintType
      obj = GravityCompensationTorqueConstraint(varargin{:});
    otherwise
      error('Drake:constructRigidBodyConstraint:UnsupportedConstraintType','The constraint type is not supported yet');
  end
end
    
end
