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
      
      if any(match(frame.poly,decomp(getPoly(obj,0)))==0)
        error('polynomial depends on variables other than t and those defined in frame');
      end
    end
    
    function V = eval(obj,t,x)  
      V = double(subs(obj.Vpoly,[obj.p_t;obj.getFrame.poly],[t;x]));
    end
    
    function Vpoly = getPoly(obj,t)
      if (nargin<2)
        Vpoly = obj.Vpoly;
      else
        Vpoly=subs(obj.Vpoly,obj.p_t,t);
      end
    end
  end
end
