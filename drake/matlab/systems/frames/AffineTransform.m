classdef AffineTransform < CoordinateTransform & AffineSystem
% represents a coordinate transform of the form x_to = T*x_from + b
  
  methods
    function obj=AffineTransform(from,to,T,b)
      obj=obj@AffineSystem([],[],[],[],[],[],[],T,b);
      obj=obj@CoordinateTransform(from,to,true,isnumeric(T)&&isnumeric(b));
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
    end
    
    function y = trajectoryOutput(obj,x,u)
      % relies on trajectory math to get through, regardless of 
      % whether C,D, and y0 are constants or trajectories.
      y=zeros(obj.num_y,1);
      if (obj.num_x) y=y+obj.C*x; end
      if (obj.num_u) y=y+obj.D*u; end
      if ~isempty(obj.y0) y=y+obj.y0; end
    end
  end
  
end
