function [c,ceq,dc,dceq] = testhsnlc(x)

c  = [];
dc = [];

ceq = [ x(1) + x(2)^2 + x(3)^3 - 3;
	x(2) - x(3)^2 + x(4) - 1;
	x(1)*x(5) - 1 ];

dceq = [ 1     2*x(2)  3*x(3)^2  0  0;
	 0     1      -2*x(3)    1  0;
	 x(5)  0       0         0  x(1)];
