classdef PolynomialTrajectory < Trajectory

  properties
    handle
    breaks
    polyframe
    p_t
  end
  
  methods 
    function obj = PolynomialTrajectory(handle,breaks,polyframe)
      error('obsolete.  trying not to use this anymore'); 
      obj = obj@Trajectory(0);
      if (nargin>0)
        if (~isa(handle,'function_handle')) error('handle should be a function handle'); end
        obj.handle=handle;
        obj.breaks = breaks;
        p = feval(handle,breaks(1));
        typecheck(p,'msspoly');
        obj.dim = size(p);
        obj.tspan = [min(breaks), max(breaks)];

        typecheck(frame,'CoordinateFrame');
        obj.polyframe = frame;

        v=decomp(p);
        t_ind = match(msspoly('t',1),v);
        if (any(t_ind)) % could be empty.. that's ok.
          obj.p_t = v(find(t_ind)); 
        end
        b = match([obj.p_t;obj.polyframe.poly],v);
        if (any(b==0)) error('polynomial contains terms that are not in the output frame'); end
      end      
    end

    function p = eval(obj,t)  
      % returns the msspoly
      sizecheck(t,1);  % only handle single time requests
      t=max(min(t,obj.tspan(end)),obj.tspan(1));
      p = obj.handle(t);
      if (~isempty(obj.p_t))
        p=subs(p,obj.p_t,t);
      end
    end
    
    function y = polyeval(obj,t,x)  
      % returns the double of the msspoly evaluated at x
      p = eval(obj,t);
      y = double(msubs(p,obj.polyframe.poly,x));
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
      y = double(msubs(p,obj.polyframe.poly,x));
    end
    
    function t = getBreaks(obj)
    % return the list of accurate/reliable points
      t = obj.breaks;
    end
        
  end
  
end

