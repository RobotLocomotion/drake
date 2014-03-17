classdef NLPtest1 < NonlinearProgram
  % NOTEST
  % nonlinear inequality constraints
  % min x2
  % s.t x1^2+4*x2^2 <= 4
  %     (x1-2)^2+x2^2 <=5
  %      x1 >= 0
  methods
    function obj = NLPtest1()
      obj = obj@NonlinearProgram(2,2,0);
      obj = obj.setNonlinearInequalityBounds(-inf(2,1),[4;5]);
      obj = obj.setVarBounds([0;-inf],[inf;inf]);
      obj = obj.setObjectiveGradientSparsity(2);
      obj = obj.setNonlinearInequalityConstraintsGradientSparsity([1;2;2;1],[1;2;1;2]);
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      f = [x(2);x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2];
      G = [0 1;2*x(1) 8*x(2);2*(x(1)-2) 2*x(2)];
    end
    
  end
end