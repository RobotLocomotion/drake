classdef LeggedRobot
  
  properties
  end
  
  methods
    function obj = LeggedRobot(obj);
      typecheck(obj,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
    end
    
  end
  
  methods(Abstract)
    planFootsteps(obj, goal, params);
  end
  
end

