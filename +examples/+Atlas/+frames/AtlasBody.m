classdef AtlasBody < SingletonCoordinateFrame
  % coordinate frame for the atlas model's rigid bodies---one coordinate per body
  % potentially useful for designating what bodies are supporting, etc.
  methods
    function obj=AtlasBody(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      obj = obj@SingletonCoordinateFrame('SupportBodies',r.getNumBodies(),'x',r.getLinkNames());
    end
  end
end
