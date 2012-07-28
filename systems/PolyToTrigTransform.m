classdef PolyToTrigTransform < CoordinateTransform
% For trig-poly dynamical systems (e.g., xdot = [x(2); sin(x(1))]), we 
% often analyze the polynomial system given where
% y=[sin(x(1);cos(x(1));x(2)];  
% This transformation converts from y to x
  
  methods
    function obj=PolyToTrigTransform(from,to,sin_ind,cos_ind,x_ind)
      obj=obj@CoordinateTransform(from,to,true,true);
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      obj=setInputFrame(obj,from);
      obj=setOutputFrame(obj,to);
      
      % now check the indices
      nx = to.dim;
      ny = from.dim;
      
      typecheck(sin_ind,'logical');
      sizecheck(sin_ind,[ny,nx]);
      typecheck(cos_ind,'logical');
      sizecheck(cos_ind,[ny,nx]);
      typecheck(x_ind,'logical');
      sizecheck(x_ind,[ny,nx]);

      % make sure that every y(i)=sin(x) has a y(j)=cos(x) and vice versa
      s=any(sin_ind,1);
      c=any(cos_ind,1);
      if ~isequal(s,c) error('every y(i)=sin(x) should have a y(j)=cos(x) and vice versa'); end
      
      obj.sin_ind = sin_ind;
      obj.cos_ind = cos_ind;
      obj.x_ind = x_ind;
    end
    
    function x=output(obj,~,~,y)
      x = atan2(obj.sin_ind'*y,obj.cos_ind'*y) + obj.x_ind'*y;
    end

  end

  properties
    sin_ind;
    cos_ind;
    x_ind;
  end
  
end
