classdef PolynomialWTimeVaryingCoefficients 
  % defines a polynomial (represented by an msspoly) with time-varying
  % coefficients (represented as a trajectory)
  
  properties
    poly;
    coeftraj;
    coefpoly;
  end

  methods
    function obj= PolynomialWTimeVaryingCoefficients(poly)
      typecheck(poly,'msspoly');
      
      [x,p,M]=decomp(poly);
      [i,j,s]=find(M);
      obj.coefpoly = msspoly('^',numel(s));
      obj.coeftraj = ConstantTrajectory(s');

      % this is just a bad way to do what I'd like to do: sparse(i,j,obj.coefpoly)
      Mpoly=msspoly(M);
      for k=1:length(i)
        Mpoly(i(k),j(k))=obj.coefpoly(k);
      end
      
      obj.poly = Mpoly*recomp(x,p);
    end
    
    function poly = getPoly(obj,t)
      poly = subs(obj.poly,obj.coefpoly,obj.coeftraj.eval(t));
    end
    
    function c = ctranspose(a)
      error('not implemented yet');
    end
    
    function c = plus(a,b)
      error('not implemented yet');
    end
    
    function c = mtimes(a,b)
      error('not implemented yet');
    end
  end
  
end
