classdef NonlinearConstraint1 < NonlinearConstraint
  %NOTEST
  methods
    function obj = NonlinearConstraint1()
      obj = obj@NonlinearConstraint([-1;-2],[3;2],4);
      obj = obj.setSparseStructure([1;1;1;1;2;2],[1;2;3;4;2;3]);
    end
    
    function [c,dc] = eval(obj,x,y)
      c = [x(1)*x(2)+y(1)+y(2); x(2)*y(1)];
      dc = [x(2) x(1) 1 1;0 y(1) x(2) 0];
    end
  end
end