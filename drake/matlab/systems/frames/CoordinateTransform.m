classdef CoordinateTransform < DrakeSystem 
    
  methods
    function obj=CoordinateTransform(from,to,feedthroughflag,tiflag)
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj=obj@DrakeSystem(0,0,from.dim,to.dim,feedthroughflag,tiflag);
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
    end
    
    function ytraj = trajectoryOutput(obj,xtraj,utraj)
      error('if it makes sense for your transform system, implement this to allow trajectories to push through (even if your system is time-varying)');
    end
  end
end
