classdef PolynomialTrajectory < Trajectory

  properties
    handle
    breaks
    p_t=[]
    p_x
  end
  
  methods 
    function obj = PolynomialTrajectory(handle,breaks)
      obj = obj@Trajectory(0);
      if (nargin>0)
        if (~isa(handle,'function_handle')) error('handle should be a function handle'); end
        obj.handle=handle;
        obj.breaks = breaks;
        p = feval(handle,breaks(1));
        typecheck(p,'msspoly');
        obj.dim = size(p);
        obj.tspan = [min(breaks), max(breaks)];
        x=decomp(p);
        t_ind = match(msspoly('t',1),x);
        if (any(t_ind)) % could be empty.. that's ok.
          obj.p_t = x(find(t_ind)); 
        end
        obj.p_x = x(find(~t_ind)); 
      end      
    end

    function p = eval(obj,t)  % returns the msspoly
      sizecheck(t,1);  % only handle single time requests
      p = obj.handle(t);
      if (~isempty(obj.p_t))
        p=subs(p,obj.p_t,t);
      end
    end
    
    function y = polyeval(obj,t,x)  % returns the msspoly evaluated at x
      p = eval(obj,t);
      y = msubs(p,obj.p_x,x);
    end
    
    function p = deriv(obj,t)
      sizecheck(t,1);  % only handle single time requests
      p = obj.handle(t);
      if (isempty(obj.p_t))  % then there is no time dependence
        p=0*p;
      else 
        p=subs(diff(p,obj.p_t),obj.p_t,t);
      end
    end
    
    function y = polyderiv(obj,t,x)
      p = deriv(obj,t);
      y = msubs(p,obj.p_x,x);
    end
    
    function t = getBreaks(obj)
    % return the list of accurate/reliable points
      t = obj.breaks;
    end
    
  end
  
end
