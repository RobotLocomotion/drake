classdef HandInput < SingletonCoordinateFrame
  % hand input coordinate frame
  methods
    function obj=HandInput(r, ind, name)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      % Inds indicate which element of the overall state frame
      % our hand state is. 
      manipInputFrame = r.getManipulator().getInputFrame();
      manipInputFrame = manipInputFrame.getFrameByNum(ind);
      input_names = manipInputFrame.getCoordinateNames();
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame(name,length(input_names),'x',input_names);
    end
  end
end
