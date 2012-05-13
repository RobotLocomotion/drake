classdef PolynomialLyapunovFunction < LyapunovFunction
  
  properties
    Vpoly;
  end
  
  methods
    function obj=PolynomialLyapunovFunction(frame,Vpoly)
      obj=obj@LyapunovFunction(frame,true);
      typecheck(Vpoly,'msspoly');
      sizecheck(Vpoly,[1 1]);  % must be a scalar
      obj.Vpoly = Vpoly;
    end
    
    function V = eval(obj,t,x)  
      V = double(subs(obj.Vpoly,obj.frame.poly,x));
    end
  end

  % todo: move over getLevelSet, getProjection, plotFunnel, ...
end
