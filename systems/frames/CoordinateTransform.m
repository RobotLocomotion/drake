classdef CoordinateTransform < DrakeSystem 
    
  methods
    function obj=CoordinateTransform(from,to,feedthroughflag,tiflag)
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj=obj@DrakeSystem(0,0,from.dim,to.dim,feedthroughflag,tiflag);
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
    end
  end
end
