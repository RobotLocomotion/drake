classdef PolynomialTrajectory 
% Like a trajectory object, but it returns an msspoly for each t (via
% function handles).  Note that it does not derive from trajectory because
% it does not make sense as a trajectory in some contents (e.g. it should
% not be a DrakeSystem)
  
% todo: replace this with a class for polynomials with Time-Varying
% Coefficients (from a trajectory).  get rid of the PolynomialTrajectory
% class completely.  see Bug 1006.

  
  properties
    handle
    breaks
    tspan
    dim
    p_t
%    p_x
  end
  
  methods 
    function obj = PolynomialTrajectory(handle,breaks)
      if (nargin>0)
        if (~isa(handle,'function_handle')) error('handle should be a function handle'); end
        obj.handle=handle;
        obj.breaks = breaks;
        p = feval(handle,breaks(1));
        typecheck(p,'msspoly');
        obj.dim = size(p);
        obj.tspan = [min(breaks), max(breaks)];
        p_t = msspoly('t',1);
%        x=decomp(p);
%        t_ind = match(p_t,x);
%        obj.p_x = x(find(~t_ind));  % x is everything except t
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
    
    function p = deriv(obj,t)
      sizecheck(t,1);  % only handle single time requests
      p = obj.handle(t);
      if (isempty(obj.p_t))  % then there is no time dependence
        p=0*p;
      else 
        p=subs(diff(p,obj.p_t),obj.p_t,t);
      end
    end
    
    function t = getBreaks(obj)
    % return the list of accurate/reliable points
      t = obj.breaks;
    end
    
    function s = size(obj,dim)
      s=obj.dim;
      if (length(s)==1) s=[s,1]; end
      if (nargin>1) s=s(dim); end
    end
        
  end
  
end

