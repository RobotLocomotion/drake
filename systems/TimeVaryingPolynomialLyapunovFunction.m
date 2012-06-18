classdef TimeVaryingPolynomialLyapunovFunction < LyapunovFunction
  
  properties
    Vtrajpoly
  end
  
  methods
    function obj = TimeVaryingPolynomialLyapunovFunction(Vtraj)
      typecheck(Vtraj,'PolynomialTrajectory');
      sizecheck(Vtraj,[1 1]);
      % construct a default frame, which I will ignore
      obj = obj@LyapunovFunction(CoordinateFrame('InvalidFrame',0,'z'),false);
    end
    
    function V = eval(obj,t,x)
      V = polyeval(obj.Vtrajpoly,x);
    end
    
    function fr = getFrame(obj)
      % ignore obj.frame field
      fr = obj.Vtrajpoly.getOutputFrame();
    end
  end
end
