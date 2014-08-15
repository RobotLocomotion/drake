classdef AtlasInput < SingletonCoordinateFrame
  % atlas input coordinate frame
  methods
    function obj=AtlasInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix     
      
      obj = obj@SingletonCoordinateFrame('AtlasInput',r.getNumInputs(),'x',input_names);
    end
  end
end
