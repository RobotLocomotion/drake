classdef CassieInput < SingletonCoordinateFrame
  methods
    function obj=CassieInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      manipInputFrame = r.getManipulator().getInputFrame();
      input_names = manipInputFrame.getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('cassieFrames.CassieInput',length(input_names),'x',input_names);
    end
  end
end
