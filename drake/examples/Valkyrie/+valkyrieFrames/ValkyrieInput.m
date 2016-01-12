classdef ValkyrieInput < SingletonCoordinateFrame
  % valkyrie input coordinate frame
  methods
    function obj=ValkyrieInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      manipInputFrame = r.getManipulator().getInputFrame();
      if (r.external_force ~= 0)
        manipInputFrame = manipInputFrame.getFrameByNum(1);
      end
      input_names = manipInputFrame.getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix

      obj = obj@SingletonCoordinateFrame('valkyrieFrames.ValkyrieInput',length(input_names),'x',input_names);
    end
  end
end
