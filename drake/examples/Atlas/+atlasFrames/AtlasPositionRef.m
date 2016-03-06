classdef AtlasPositionRef < SingletonCoordinateFrame
  % atlas position reference input frame
  methods
    function obj=AtlasPositionRef(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      input_names = r.getInputFrame().getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('atlasFrames.AtlasPositionRef',r.getNumInputs(),'x',input_names)
    end
  end
end
