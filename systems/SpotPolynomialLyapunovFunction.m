classdef SpotPolynomialLyapunovFunction < PolynomialLyapunovFunction
  
  properties
    Vpoly;
  end
  
  methods 
    function obj = SpotPolynomialLyapunovFunction(frame,Vpoly)
      obj = obj@PolynomialLyapunovFunction(frame,true);
      
      typecheck(Vpoly,'msspoly');
      sizecheck(Vpoly,[1 1]);  % must be a scalar
      obj.Vpoly=Vpoly;
      if (deg(Vpoly,obj.p_t)>0) obj.time_invariant_flag = false; end
      
      if any(match(frame.getPoly,decomp(getPoly(obj,0)))==0)
        error('polynomial depends on variables other than t and those defined in frame');
      end
    end
    
    function V = eval(obj,t,x)  
      V = double(subs(obj.Vpoly,[obj.p_t;obj.getFrame.getPoly],[t;x]));
    end
    
    function Vpoly = getPoly(obj,t)
      if (nargin<2)
        Vpoly = obj.Vpoly;
      else
        Vpoly=subs(obj.Vpoly,obj.p_t,t);
      end
    end
    
    function b = mtimes(a,b)
      % support simple scaling of Lyapunov functions via multiplication by
      % a (scalar) double
      if ~isa(b,'PolynomialLyapunovFunction')
        % then a must be the lyapunov function.  swap them.
        tmp=a; a=b; b=tmp;
      end
      typecheck(a,'numeric');
      sizecheck(a,1);

      b.Vpoly = a*b.Vpoly;
    end
  end
end
