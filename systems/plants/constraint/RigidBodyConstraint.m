classdef RigidBodyConstraint
  % @param category      -- All constraints in the same category share the same function
  %                         interface. Please use negative number for this category
  % @param type          -- Only for non-abstract constraint. Each non-abstract constraint
  %                         class has a unique type. Please use positive number for
  %                         this category.
  properties(SetAccess = protected)
    category
    type
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
  end
  
  methods
    function obj = RigidBodyConstraint(category)
      if(~isnumeric(category))
        error('Drake:Constraint:type has to be an integer');
      end
      category = floor(category);
      if(category<-7||category>-1)
        error('Drake:Constraint: Currently type can only be within [-6 -1]');
      end
      obj.category = category;
      obj.type = 0;
    end
  end
end