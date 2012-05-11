classdef AffineTransform < CoordinateTransform & AffineSystem
% represents a coordinate transform of the form x_to = T*x_from + b
  
  methods
    function obj=AffineTransform(from,to,T,b)
      obj=obj@AffineSystem([],[],[],[],[],[],[],T,b);
      obj=obj@CoordinateTransform(from,to);
    end
  end
  
end
