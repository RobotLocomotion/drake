classdef AtlasInput < SingletonCoordinateFrame
  % atlas input coordinate frame
  methods
    function obj=AtlasInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      manipInputFrame = r.getManipulator().getInputFrame();
      if (r.hands > 0)
        manipInputFrame = manipInputFrame.getFrameByNum(1);
      end
      input_names = manipInputFrame.coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('AtlasInput',length(input_names),'x',input_names);
    end
  end
end
