classdef LeggedRobot

  properties(Abstract)
    fixed_point_file
  end

  methods
    function obj = LeggedRobot(obj)
      typecheck(obj,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
    end

  end

  methods(Abstract)
    planFootsteps(obj, goal, params);
  end

  methods
    function xstar = loadFixedPoint(obj)
      load(obj.fixed_point_file, 'xstar');
    end
  end


end

