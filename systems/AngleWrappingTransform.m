classdef AngleWrappingTransform  < CoordinateTransform
  % wraps all coordinates with angle_flag = true to be inside
  % [-pi+q0,pi+q0]
  
  properties 
    angle_flag;
    q0;
  end
  
  methods
    function obj = AngleWrappingTransform(from,to,angle_flag,q0)
      typecheck(from,'CoordinateFrame');
      typecheck(to,'CoordinateFrame');
      if (from.dim ~= to.dim) 
        error('input and output frames must have the same dimension'); 
      end
      obj = obj@CoordinateTransform(from,to,true,true);
      
      sizecheck(angle_flag,[from.dim,1]);
      obj.angle_flag = logical(angle_flag);
      if (nargin<4) 
        q0 = zeros(sum(obj.angle_flag),1); 
      else
        % todo: consider adding support for q0 being a trajectory
        if ~sizecheck(q0,[sum(obj.angle_flag),1])
          error('q0 must be the size of the number of true elements in angle_flag');
        end
      end
      obj.q0 = q0;
      obj = setNumZeroCrossings(obj,sum(obj.angle_flag));
    end
    
    function y = output(obj,~,~,u)
      y = u;
      y(obj.angle_flag) = mod(u(obj.angle_flag)-obj.q0+pi,2*pi)+obj.q0-pi;      
    end
    
    % add zero-crossings because the mod causes a discontinuity
    function zcs = zeroCrossings(obj,~,~,u)
      zcs = cos((u(obj.angle_flag)-obj.q0)/2);  % cos(a/2) crosses zero at pi,3*pi,5*pi, etc.
    end
    
  end
end