function [obj,grad] = testobj(x)

obj  = -x(1)*x(2)*x(3);
grad = [ -x(2)*x(3)  -x(1)*x(3)  -x(1)*x(2) ];
