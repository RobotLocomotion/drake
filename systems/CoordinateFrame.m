classdef CoordinateFrame < handle
% Every input, state, and output in a RobotLibSystem has a coordinate frame
% attached to it.  Many bugs can be avoided by forcing developers to be 
% explicit about these coordinate systems when they make combinations of
% systems.
  
  properties
    name
    transforms={};
  end
  
  methods
    function obj=CoordinateFrame(name)
      typecheck(name,'char');
      obj.name = name;
    end
  end
  
  % todo: consider putting LCM encode/decode in here
end
