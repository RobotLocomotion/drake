classdef RigidBodyConstraint
  % @param category      -- All constraints in the same category share the same function
  %                         interface. Please use negative number for this category
  % @param type          -- Only for non-abstract constraint. Each non-abstract constraint
  %                         class has a unique type. Please use positive number for
  %                         this category.
  % @param robot         -- A RigidBodyManipulator or TimeSteppingRigidBodyManipulator
  % @param mex_ptr       -- A DrakeConstraintMexPointer. The mex pointer of the
  % RigidBodyConstraint
  properties(SetAccess = protected)
    category
    type
    robot
    mex_ptr=0;
  end

  properties(Constant)
    SingleTimeKinematicConstraintCategory = -1;
    MultipleTimeKinematicConstraintCategory = -2;
    QuasiStaticConstraintCategory = -3;
    PostureConstraintCategory = -4;
    MultipleTimeLinearPostureConstraintCategory = -5;
    SingleTimeLinearPostureConstraintCategory = -6;
    ContactWrenchConstraintCategory = -7;
  end

  properties(Constant)
    QuasiStaticConstraintType = 1;
    PostureConstraintType = 2;
    SingleTimeLinearPostureConstraintType = 3;
    AllBodiesClosestDistanceConstraintType = 4;
    WorldEulerConstraintType = 5;
    WorldGazeDirConstraintType = 6;
    WorldGazeOrientConstraintType = 7;
    WorldGazeTargetConstraintType = 8;
    RelativeGazeTargetConstraintType = 9;
    WorldCoMConstraintType = 10;
    WorldPositionConstraintType = 11;
    WorldPositionInFrameConstraintType = 12;
    WorldQuatConstraintType = 13;
    Point2PointDistanceConstraintType = 14;
    Point2LineSegDistConstraintType = 15;
    WorldFixedPositionConstraintType = 16;
    WorldFixedOrientConstraintType = 17;
    WorldFixedBodyPoseConstraintType = 18;
    PostureChangeConstraintType = 19;
    RelativePositionConstraintType = 20;
    FrictionConeWrenchConstraintType = 21;
    LinearFrictionConeWrenchConstraintType = 22;
    RailGraspWrenchConstraintType = 23;
    RelativeQuatConstraintType = 24;
    RelativeGazeDirConstraintType = 25;
    MinDistanceConstraintType = 26;
    GravityCompensationTorqueConstraintType = 27;
  end

  methods
    function obj = RigidBodyConstraint(category,robot)
      if(~isnumeric(category))
        error('Drake:RigidBodyConstraint:BadInput','category has to be an integer');
      end
      category = floor(category);
      if(category<-7||category>-1)
        error('Drake:RigidBodyConstraint:BadInput','Unsupported constraint category');
      end
      obj.category = category;
      obj.type = 0;
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('Drake:RigidBodyConstraint:BadInput','robot has to be a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      obj.robot = robot;
    end

    function cat_str = categoryString(obj)
      if(obj.category == RigidBodyConstraint.SingleTimeKinematicConstraintCategory)
        cat_str = 'SingleTimeKinematicConstraint';
      elseif(obj.category == RigidBodyConstraint.MultipleTimeKinematicConstraintCategory)
        cat_str = 'MultipleTimeKinematicConstraint';
      elseif(obj.category == RigidBodyConstraint.QuasiStaticConstraintCategory)
        cat_str = 'QuasiStaticConstraint';
      elseif(obj.category == RigidBodyConstraint.PostureConstraintCategory)
        cat_str = 'PostureConstraint';
      elseif(obj.category == RigidBodyConstraint.MultipleTimeLinearPostureConstraintCategory)
        cat_str = 'MultipleTimeLinearPostureConstraint';
      elseif(obj.category == RigidBodyConstraint.SingleTimeLinearPostureConstraintCategory)
        cat_str = 'SingleTimeLinearPostureConstraint';
      else
        error('Drake:RigidBody:UnsupportedCategory','The constraint category is not supported');
      end
    end

  end

  methods(Abstract)
    cnstr = generateConstraint(obj,t)
    % Given the time, this method would generate a cell of Constraint objects, that encode
    % the numerical constraint at the given time
  end
end
