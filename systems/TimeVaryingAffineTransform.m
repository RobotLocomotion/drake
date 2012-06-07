classdef TimeVaryingAffineTransform < CoordinateTransform & TimeVaryingAffineSystem
% represents a coordinate transform of the form x_to = T(t)*x_from + b(t)
  
  methods
    function obj=TimeVaryingAffineTransform(from,to,T,b)
      obj=obj@TimeVaryingAffineSystem([],[],[],[],[],[],[],T,b);
      obj=obj@CoordinateTransform(from,to,true,false);
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
    end
  end
  
end
