classdef IRB140State < SingletonCoordinateFrame
  
  methods
    function obj=IRB140State(r)
      typecheck(r,{'TimeSteppingRigidBodyManipulator','RigidBodyManipulator'});
      manipStateFrame = r.getManipulator().getStateFrame();
      if (r.hands > 0)
        manipStateFrame = manipStateFrame.getFrameByNum(1);
      end
      coordinates = manipStateFrame.getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('IRB140State',length(coordinates),'x',coordinates);
    end
  end
end
