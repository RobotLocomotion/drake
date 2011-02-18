classdef LCMCoder
% Interface for classes which encode and decode robot-specific messages from LCM.

  methods 
    function obj = LCMCoder(dim_x,dim_u,dim_y)
      if (nargin>0)
        obj.dim_x = dim_x;
        obj.dim_u = dim_u;
        obj.dim_y = dim_y;
      end
    end
  end

  methods (Abstract=true)
    str = getRobotName(obj)
    
    msg = encodeX(obj,t,x)
    [x,t] = decodeX(obj,msg)

    msg = encodeU(obj,t,u)
    [u,t] = decodeU(obj,msg)

    msg = encodeY(obj,t,y)
    [y,t] = decodeY(obj,msg)
  end
  
  properties
    dim_x = 0;  % length of the x vector
    dim_u = 0;  % length of the u vector
    dim_y = 0;  % length of the y vector
  end
  
end
