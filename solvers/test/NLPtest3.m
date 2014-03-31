classdef NLPtest3 < NonlinearProgram
  % NOTEST
  % nonlinear inequality constraints, nonlinear equality constraints, linear inequality
  % constraints and linear equality constraints
  % min x2^2+x1*x3+x3
  % x1^2+4*x2^2<=4
  % (x1-2)^2+x2^2<=5
  % x1 >= 0
  % -1<=x1+2*x3 <=10
  % x1+3*x3 = 0
  % x1*x2*x3 = 1/6;
  % -10<=x2^2+x2*x3+2*x3^2<=30
  % x2+x3<=10
  % -x2+x3 = 0.1;
  methods
    function obj = NLPtest3()
      obj = obj@NonlinearProgram(3,3,1);
      obj = obj.setVarBounds([0;-inf;-inf],inf(3,1));
      obj = obj.addLinearEqualityConstraints([1 0 3;0 -1 1],[0;0.1]);
      obj = obj.addLinearInequalityConstraints([1 2 0;-1 -2 0;0 1 1],[10;1;10]);
      obj = obj.setNonlinearInequalityConstraintsGradientSparsity([1;1;2;2;3;3],[1;2;1;2;2;3]);
      obj = obj.setNonlinearEqualityConstraintsGradientSparsity([1;1;1],[1;2;3]);
      obj = obj.setNonlinearInequalityBounds([-inf;-inf;-10],[4;5;30]);
    end
    
    function [f,G] = objectiveAndNonlinearConstraints(obj,x)
      f = [x(2)^2+x(1)*x(3)+x(3);x(1)^2+4*x(2)^2;(x(1)-2)^2+x(2)^2;x(2)^2+x(2)*x(3)+x(3)^2;x(1)*x(2)*x(3)-1/6];
      G = [x(3) 2*x(2) x(1)+1;2*x(1) 8*x(2) 0;2*(x(1)-2) 2*x(2) 0; 0 2*x(2)+x(3) x(2)+2*x(3); x(2)*x(3) x(1)*x(3) x(1)*x(2);];
    end
  end
end