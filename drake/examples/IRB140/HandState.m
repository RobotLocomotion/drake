classdef HandState < SingletonCoordinateFrame
  
  methods
    function obj=HandState(r, ind, name)
      typecheck(r,{'TimeSteppingRigidBodyManipulator','RigidBodyManipulator'});
      % Inds indicate which element of the overall state frame
      % our hand state is. 
      manipStateFrame = r.getManipulator().getStateFrame();
      manipStateFrame = manipStateFrame.getFrameByNum(ind);
      coordinates = manipStateFrame.getCoordinateNames();
      obj = obj@SingletonCoordinateFrame(name,length(coordinates),'x',coordinates);
    end
  end
end
