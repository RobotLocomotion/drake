classdef TrigToPolyTransform < CoordinateTransform
% For trig-poly dynamical systems (e.g., xdot = [x(2); sin(x(1))]), we 
% often analyze the polynomial system given where
% y=[sin(x(1);cos(x(1));x(2)];  
% This transformation converts from x to y
  
  methods
    function obj=TrigToPolyTransform(from,to,sin_ind,cos_ind,x_ind)
      obj=obj@CoordinateTransform(from,to,true,true);
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
      
      % now check the indices
      nx = from.dim;
      ny = to.dim;
      
      typecheck(sin_ind,'logical');
      sizecheck(sin_ind,[ny,nx]);
      typecheck(cos_ind,'logical');
      sizecheck(cos_ind,[ny,nx]);
      typecheck(x_ind,'logical');
      sizecheck(x_ind,[ny,nx]);
      
      obj.sin_ind = sin_ind;
      obj.cos_ind = cos_ind;
      obj.x_ind = x_ind;
    end
    
    function y=output(obj,~,~,x)
      y = obj.sin_ind*sin(x) + obj.cos_ind*cos(x) + obj.x_ind*x;
    end
  end

  properties
    sin_ind;
    cos_ind;
    x_ind;
  end
  
end
