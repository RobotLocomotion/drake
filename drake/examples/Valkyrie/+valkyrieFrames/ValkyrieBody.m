classdef ValkyrieBody < SingletonCoordinateFrame
  % coordinate frame for the valkyrie model's rigid bodies---one coordinate per body
  % potentially useful for designating what bodies are supporting, etc.
  methods
    function obj=ValkyrieBody(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      obj = obj@SingletonCoordinateFrame('SupportBodies',r.getNumBodies(),'x',r.getLinkNames());
    end
  end
end
