classdef NLPtest2 < NonlinearProgram
% NOTEST
% nonlinear equality and inequality constraints, only one feasible point
% min  x2^2
% s.t x1^2+4x2^2<=4
% s.t (x1-2)^2+x2^2<=5
%      x1x2 = 1
%      x1>=0
methods
  function obj = NLPtest2()
    obj = obj@NonlinearProgram(2,2,1);
    obj = obj.setNonlinearInequalityBounds(-inf(2,1),[4;5]);
    obj = obj.setVarBounds([0;-inf],[inf;inf]);
    obj = obj.setObjectiveGradientSparsity(2);
    obj = obj.setNonlinearInequalityConstraintsGradientSparsity([1;2;2;1],[1;2;1;2]);
    obj = obj.setNonlinearEqualityConstraintsGradientSparsity([1;1],[2;1]);  
  end
  
  function [f,df] = objective(obj,x)
    f = x(2)^2;
    df = [0 2*x(2)];
  end
  
  function [g,h,dg,dh] = nonlinearConstraints(obj,x)
    g = [x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2];
    h = x(1)*x(2)-1;
    dg = [2*x(1) 8*x(2);2*(x(1)-2) 2*x(2)];
    dh = [x(2) x(1)];
  end
end
end