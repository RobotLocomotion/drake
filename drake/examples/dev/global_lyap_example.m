

x=msspoly('x',3);
xdot = [ -x(1)^3 - x(2)*x(3) - x(1) - x(1)*x(3)^2; ...
         -x(1)*x(3) + 2*x(1)^3 - x(2); ...
         -x(3) + 2*x(1)^2 ]
V = .5*x'*x

Vdot = diff(V,x)*xdot


prog=mssprog;
slack=msspoly('s');
prog.free=slack;
prog.sos=-Vdot-slack*x'*x;

prog.sedumi=-slack;

slack=double(prog(slack))
c = double(prog(c))


