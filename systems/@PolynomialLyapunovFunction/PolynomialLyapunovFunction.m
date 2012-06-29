classdef PolynomialLyapunovFunction < LyapunovFunction
  
  properties (SetAccess=private,GetAccess=private)
    Vpoly;
    Vpolytraj;
    p_t
  end
  
  methods
    function obj=PolynomialLyapunovFunction(frame,Vpoly)
      obj=obj@LyapunovFunction(frame,true);
      obj.p_t = msspoly('t',1);
      sizecheck(Vpoly,[1 1]);  % must be a scalar
      if (isa(Vpoly,'msspoly'))
        obj.Vpoly=Vpoly;
        if (deg(Vpoly,obj.p_t)>0) obj.time_invariant_flag = false; end
      else
        typecheck(Vpoly,'PolynomialTrajectory');
        obj.Vpolytraj = Vpoly;
        obj.time_invariant_flag = false;
      end
      
      if any(match(frame.poly,decomp(getPoly(obj)))==0)
        error('polynomial depends on variables other than t and those defined in frame');
      end
    end
    
    function V = eval(obj,t,x)  
      if isempty(obj.Vpoly)
        V = double(subs(obj.Vpolytraj.eval(t),[obj.p_t;obj.frame.poly],[t;x]));
      else
        V = double(subs(obj.Vpoly,[obj.p_t;obj.frame.poly],[t;x]));
      end
    end
    
    function Vpoly = getPoly(obj,t)
      if isempty(obj.Vpoly)
        if (nargin<2) t=obj.Vpolytraj.tspan(1); end
        Vpoly = subs(obj.Vpolytraj.eval(t),obj.p_t,t);
      else
        if (nargin<2) t=0; end
        Vpoly=subs(obj.Vpoly,obj.p_t,t);
      end
    end
  end

  % todo: move over getLevelSet, getProjection, plotFunnel, ...
end
