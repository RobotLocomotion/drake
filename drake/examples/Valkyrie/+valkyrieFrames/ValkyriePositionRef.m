classdef ValkyriePositionRef < SingletonCoordinateFrame
  % valkyrie position reference input frame
  methods
    function obj=ValkyriePositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      input_names = r.getInputFrame().getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('valkyrieFrames.ValkyriePositionRef',r.getNumInputs(),'x',input_names)
    end
  end
end
