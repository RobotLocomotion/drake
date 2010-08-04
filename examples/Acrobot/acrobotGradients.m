function df = acrobotGradients(r,t,x,u,order)

% This file evaluates the gradients of the robot dynamics.
% Usage:  df = acrobot_gradients(r,t,x,u,order)
%   r 		- acrobot class object
%   t 		- time (scalar)
%   x 		- 4-by-1 state vector
%   u 		- 1-by-1 input vector
%   order 	- order of gradients to compute (1<=order<=3)
% This is an auto-generated file.  See <a href="matlab: help robot>generate_dynamics_func_gradients">robot>generate_dynamics_func_gradients</a>. 

% NOTEST  % this tag tells my runAllTests script that this file can't be run as a unit test


% Check inputs:
if (~isa(r,'AcrobotDynamics')) error(' r must be an AcrobotDynamics class object'); end
if (length(t)~=1) error('t must be a scalar'); end
if (any(size(x)~=[4,1])) error('x must be a 4-by-1 vector'); end
if (any(size(u)~=[1,1])) error('u must be a 1-by-1 vector'); end
if ((order<1) || (order>3)) error('1<=order<=3'); end

% Symbol table:
l1=r.l1; l2=r.l2; m1=r.m1; m2=r.m2; g=r.g; b1=r.b1; b2=r.b2; lc1=r.lc1; lc2=r.lc2; I1=r.I1; I2=r.I2; xG=r.xG; uG=r.uG;
x1=x(1);x2=x(2);x3=x(3);x4=x(4);
u1=u(1);

% df{1}
a = sparse(4,6);
a(1,4) = 1;
a(2,5) = 1;
a(3,2) = -(l1*(I2*g*m2*cos(x1) - g*lc2^2*m2^2*cos(x1 + x2)*cos(x2)) + I2*(g*lc2*m2*cos(x1 + x2) + g*lc1*m1*cos(x1)) - I2*g*lc2*m2*cos(x1 + x2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2);
a(3,3) = ((I2*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - (I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (I2*(l1*lc2*m2*cos(x2)*x4^2 + 2*l1*lc2*m2*x3*cos(x2)*x4 - g*lc2*m2*cos(x1 + x2)) + (l1*lc2*m2*cos(x2)*x3^2 + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - l1*lc2*m2*sin(x2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(3,4) = (sin(x2)*(2*I2*l1*lc2*m2*x3 + 2*I2*l1*lc2*m2*x4) - I2*b1 + l1^2*lc2^2*m2^2*x3*sin(2*x2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(3,5) = (I2*b2 + l1*lc2*m2*(I2*(2*x3*sin(x2) + 2*x4*sin(x2)) + b2*cos(x2)))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2);
a(3,6) = (I2 + l1*lc2*m2*cos(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,2) = -((g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,3) = ((I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*cos(x2)*x4^2 + 2*l1*lc2*m2*x3*cos(x2)*x4 - g*lc2*m2*cos(x1 + x2)) + (l1*lc2*m2*cos(x2)*x3^2 + g*lc2*m2*cos(x1 + x2))*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - 2*l1*lc2*m2*sin(x2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)) + l1*lc2*m2*sin(x2)*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) + ((2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*((m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)) - (I2 + l1*lc2*m2*cos(x2))*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2))))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,4) = -((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,5) = (b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,6) = -(I1 + I2 + m2*(l1^2 + 2*lc2*cos(x2)*l1))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
df{1} = a;


if (order>=3)
% df{2}{1}
a = sparse(4,6);
df{2}{1} = a;

% df{2}{2}
a = sparse(4,6);
a(3,2) = -((g*l1*lc2^2*m2^2*sin(x1))/2 - I2*g*l1*m2*sin(x1) - I2*g*lc1*m1*sin(x1) + (g*l1*lc2^2*m2^2*sin(x1 + 2*x2))/2)/((- cos(2*x2)/2 - 1/2)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(3,3) = (g*l1*lc2^2*m2^2*((l1^2*lc2^2*m2^2*sin(x1))/2 + (l1^2*lc2^2*m2^2*sin(x1 - 2*x2))/4 + (l1^2*lc2^2*m2^2*sin(x1 + 2*x2))/4) - I2*g*l1*lc2^2*m2^2*(I1*sin(x1 + 2*x2) + (l1^2*m2*sin(x1 - 2*x2))/2 + (l1^2*m2*sin(x1 + 2*x2))/2 + (l1*lc1*m1*sin(x1 - 2*x2))/2 - (l1*lc1*m1*sin(x1 + 2*x2))/2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
a(4,2) = ((I2 + l1*lc2*m2*cos(x2))*(g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - g*lc2*m2*sin(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,3) = - (g*lc2*m2*sin(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - g*lc2*m2*sin(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) - l1*lc2*m2*sin(x2)*(g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2)) + 2*g*l1*lc2^2*m2^2*cos(x1 + x2)*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (((g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
df{2}{2} = a;

% df{2}{3}
a = sparse(4,6);
a(3,2) = (2*g*l1*lc2^2*m2^2*(l1^2*lc2^2*m2^2*sin(x1) + (l1^2*lc2^2*m2^2*sin(x1 - 2*x2))/2 + (l1^2*lc2^2*m2^2*sin(x1 + 2*x2))/2) - 2*I2*g*l1*lc2^2*m2^2*(2*I1*sin(x1 + 2*x2) + l1^2*m2*sin(x1 - 2*x2) + l1^2*m2*sin(x1 + 2*x2) + l1*lc1*m1*sin(x1 - 2*x2) - l1*lc1*m1*sin(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(3,3) = (I2*(l1*lc2*m2*sin(x2)*x4^2 + 2*l1*lc2*m2*x3*sin(x2)*x4 - g*lc2*m2*sin(x1 + x2)) + (I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*sin(x2)*x3^2 + g*lc2*m2*sin(x1 + x2)) + 2*l1*lc2*m2*sin(x2)*(l1*lc2*m2*cos(x2)*x3^2 + g*lc2*m2*cos(x1 + x2)) + l1*lc2*m2*cos(x2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(I2*(l1*lc2*m2*cos(x2)*x4^2 + 2*l1*lc2*m2*x3*cos(x2)*x4 - g*lc2*m2*cos(x1 + x2)) + (l1*lc2*m2*cos(x2)*x3^2 + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - l1*lc2*m2*sin(x2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2))))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - ((I2*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - (I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 + (2*(I2*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - (I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3;
a(3,4) = - ((2*I2*l1*lc2*m2*x3 + 2*I2*l1*lc2*m2*x4)*(2*sin(x2/2)^2 - 1) + 2*l1^2*lc2^2*m2^2*x3*(2*sin(x2)^2 - 1))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2) - (l1^2*lc2^2*m2^2*sin(2*x2)*(sin(x2)*(2*I2*l1*lc2*m2*x3 + 2*I2*l1*lc2*m2*x4) - I2*b1 + l1^2*lc2^2*m2^2*x3*sin(2*x2)))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2;
a(3,5) = (l1*lc2*m2*(I2*(2*x3*cos(x2) + 2*x4*cos(x2)) - b2*sin(x2)))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2) - (2*l1^2*lc2^2*m2^2*cos(x2)*sin(x2)*(I2*b2 + l1*lc2*m2*(I2*(2*x3*sin(x2) + 2*x4*sin(x2)) + b2*cos(x2))))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(3,6) = (l1^3*lc2^3*m2^3*cos(x2)^2*sin(x2) + I2*l1*lc2*m2*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,2) = - (g*lc2*m2*sin(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - g*lc2*m2*sin(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) - l1*lc2*m2*sin(x2)*(g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2)) + 2*g*l1*lc2^2*m2^2*cos(x1 + x2)*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (((g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,3) = (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2*((m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)) - (I2 + l1*lc2*m2*cos(x2))*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2))))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 - (((m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)) - (I2 + l1*lc2*m2*cos(x2))*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - ((I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*sin(x2)*x4^2 + 2*l1*lc2*m2*x3*sin(x2)*x4 - g*lc2*m2*sin(x1 + x2)) + (l1*lc2*m2*sin(x2)*x3^2 + g*lc2*m2*sin(x1 + x2))*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*sin(x2)*(l1*lc2*m2*cos(x2)*x4^2 + 2*l1*lc2*m2*x3*cos(x2)*x4 - g*lc2*m2*cos(x1 + x2)) + 4*l1*lc2*m2*sin(x2)*(l1*lc2*m2*cos(x2)*x3^2 + g*lc2*m2*cos(x1 + x2)) + 2*l1*lc2*m2*cos(x2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)) - l1*lc2*m2*cos(x2)*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) + (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*((I2 + l1*lc2*m2*cos(x2))*(l1*lc2*m2*cos(x2)*x4^2 + 2*l1*lc2*m2*x3*cos(x2)*x4 - g*lc2*m2*cos(x1 + x2)) + (l1*lc2*m2*cos(x2)*x3^2 + g*lc2*m2*cos(x1 + x2))*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - 2*l1*lc2*m2*sin(x2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)) + l1*lc2*m2*sin(x2)*(b1*x3 - 2*l1*lc2*m2*x3*sin(x2)*x4 - l1*lc2*m2*sin(x2)*x4^2 + g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2))))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,4) = (l1*lc2*m2*sin(x2)*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 4*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*l1*lc2*m2*x3*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*x4*cos(x2)*(I2 + l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,5) = ((b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) - (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) + 2*b2*l1*lc2*m2*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,6) = -(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2))*(m2*l1^2 + lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
df{2}{3} = a;

% df{2}{4}
a = sparse(4,6);
a(3,3) = ((I2*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (2*l1*lc2*m2*x3*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*I2*l1*lc2*m2*x4*cos(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(3,4) = (sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(3,5) = (2*I2*l1*lc2*m2*sin(x2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(4,3) = (l1*lc2*m2*sin(x2)*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 4*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*l1*lc2*m2*x3*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*x4*cos(x2)*(I2 + l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,4) = -(2*l1^2*lc2^2*m2^2*sin(2*x2) + 2*l1*lc2*m2*sin(x2)*(m2*l1^2 + I1 + I2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(4,5) = -(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
df{2}{4} = a;

% df{2}{5}
a = sparse(4,6);
a(3,3) = - (I2*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) - b2*l1*lc2*m2*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - ((I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(3,4) = (2*I2*l1*lc2*m2*sin(x2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(3,5) = (2*I2*l1*lc2*m2*sin(x2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(4,3) = ((b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) - (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) + 2*b2*l1*lc2*m2*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,4) = -(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(4,5) = -(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
df{2}{5} = a;

% df{2}{6}
a = sparse(4,6);
a(3,3) = (l1^3*lc2^3*m2^3*cos(x2)^2*sin(x2) + I2*l1*lc2*m2*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = -(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2))*(m2*l1^2 + lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
df{2}{6} = a;

end  % if (order>=3)

if (order>=3)
% df{3}{1}{1}
a = sparse(4,6);
df{3}{1}{1} = a;

% df{3}{1}{2}
a = sparse(4,6);
df{3}{1}{2} = a;

% df{3}{1}{3}
a = sparse(4,6);
df{3}{1}{3} = a;

% df{3}{1}{4}
a = sparse(4,6);
df{3}{1}{4} = a;

% df{3}{1}{5}
a = sparse(4,6);
df{3}{1}{5} = a;

% df{3}{1}{6}
a = sparse(4,6);
df{3}{1}{6} = a;

% df{3}{2}{1}
a = sparse(4,6);
df{3}{2}{1} = a;

% df{3}{2}{2}
a = sparse(4,6);
a(3,2) = -((g*l1*lc2^2*m2^2*cos(x1))/2 - I2*g*l1*m2*cos(x1) - I2*g*lc1*m1*cos(x1) + (g*l1*lc2^2*m2^2*cos(x1 + 2*x2))/2)/((- cos(2*x2)/2 - 1/2)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(3,3) = (g*l1*lc2^2*m2^2*((l1^2*lc2^2*m2^2*cos(x1))/2 + (l1^2*lc2^2*m2^2*cos(x1 - 2*x2))/4 + (l1^2*lc2^2*m2^2*cos(x1 + 2*x2))/4) - I2*g*l1*lc2^2*m2^2*(I1*cos(x1 + 2*x2) + (l1^2*m2*cos(x1 - 2*x2))/2 + (l1^2*m2*cos(x1 + 2*x2))/2 + (l1*lc1*m1*cos(x1 - 2*x2))/2 - (l1*lc1*m1*cos(x1 + 2*x2))/2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
a(4,2) = ((g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,3) = (((I2 + l1*lc2*m2*cos(x2))*(g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - g*lc2*m2*sin(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - g*lc2*m2*cos(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) + l1*lc2*m2*sin(x2)*(g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - 2*g*l1*lc2^2*m2^2*sin(x1 + x2)*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
df{3}{2}{2} = a;

% df{3}{2}{3}
a = sparse(4,6);
a(3,2) = -(2*g*l1*lc2^2*m2^2*(2*I1*I2*cos(x1 + 2*x2) - l1^2*lc2^2*m2^2*cos(x1) - (l1^2*lc2^2*m2^2*cos(x1 - 2*x2))/2 - (l1^2*lc2^2*m2^2*cos(x1 + 2*x2))/2 + I2*l1^2*m2*cos(x1 - 2*x2) + I2*l1^2*m2*cos(x1 + 2*x2) + I2*l1*lc1*m1*cos(x1 - 2*x2) - I2*l1*lc1*m1*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(3,3) = (2*g*l1*lc2^2*m2^2*(l1^2*lc2^2*m2^2*cos(x1 + 2*x2) - l1^2*lc2^2*m2^2*cos(x1 - 2*x2) - 4*I1*I2*cos(x1 + 2*x2) + 2*I2*l1^2*m2*cos(x1 - 2*x2) - 2*I2*l1^2*m2*cos(x1 + 2*x2) + 2*I2*l1*lc1*m1*cos(x1 - 2*x2) + 2*I2*l1*lc1*m1*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 + (4*g*l1^3*lc2^4*m2^4*sin(2*x2)*(4*I1*I2*sin(x1 + 2*x2) + 2*I2*l1^2*m2*sin(x1 - 2*x2) + 2*I2*l1^2*m2*sin(x1 + 2*x2) - 2*l1^2*lc2^2*m2^2*sin(x1) - l1^2*lc2^2*m2^2*sin(x1 - 2*x2) - l1^2*lc2^2*m2^2*sin(x1 + 2*x2) + 2*I2*l1*lc1*m1*sin(x1 - 2*x2) - 2*I2*l1*lc1*m1*sin(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
a(4,2) = (g*lc2*m2*(2*I1*cos(x1 + x2) + l1^2*m2*cos(x1 + x2) + l1^2*m2*cos(x1 - x2) - l1*lc1*m1*cos(x1 + x2) + l1*lc1*m1*cos(x1 - x2) + 2*l1*lc2*m2*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2)) + (2*g*l1^2*lc2^2*m2^2*sin(2*x2)*(2*I2*l1*m2*sin(x1) - 2*I1*lc2*m2*sin(x1 + x2) - l1*lc2^2*m2^2*sin(x1) - l1^2*lc2*m2^2*sin(x1 + x2) + 2*I2*lc1*m1*sin(x1) + l1^2*lc2*m2^2*sin(x1 - x2) - l1*lc2^2*m2^2*sin(x1 + 2*x2) + l1*lc1*lc2*m1*m2*sin(x1 + x2) + l1*lc1*lc2*m1*m2*sin(x1 - x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(4,3) = (g*lc2*m2*(2*I1*cos(x1 + x2) + l1^2*m2*cos(x1 + x2) - l1^2*m2*cos(x1 - x2) - l1*lc1*m1*cos(x1 + x2) - l1*lc1*m1*cos(x1 - x2) + 4*l1*lc2*m2*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2)) - (16*g*l1^4*lc2^4*m2^4*(cos(4*x2)/2 - 1/2)*(I2*l1*m2*cos(x1) - I1*lc2*m2*cos(x1 + x2) - l1^2*lc2*m2^2*cos(x1 + x2) + I2*lc1*m1*cos(x1) + l1^2*lc2*m2^2*cos(x1)*cos(x2) - l1*lc2^2*m2^2*cos(x1 + x2)*cos(x2) + l1*lc1*lc2*m1*m2*cos(x1)*cos(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3 - (8*g*l1^2*lc2^2*m2^2*cos(2*x2)*(I2*l1*m2*cos(x1) - I1*lc2*m2*cos(x1 + x2) - l1^2*lc2*m2^2*cos(x1 + x2) + I2*lc1*m1*cos(x1) + l1^2*lc2*m2^2*cos(x1)*cos(x2) - l1*lc2^2*m2^2*cos(x1 + x2)*cos(x2) + l1*lc1*lc2*m1*m2*cos(x1)*cos(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 - (4*g*l1^2*lc2^3*m2^3*sin(2*x2)*(2*I1*sin(x1 + x2) + l1^2*m2*sin(x1 + x2) + l1^2*m2*sin(x1 - x2) - l1*lc1*m1*sin(x1 + x2) + l1*lc1*m1*sin(x1 - x2) + 2*l1*lc2*m2*sin(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
df{3}{2}{3} = a;

% df{3}{2}{4}
a = sparse(4,6);
df{3}{2}{4} = a;

% df{3}{2}{5}
a = sparse(4,6);
df{3}{2}{5} = a;

% df{3}{2}{6}
a = sparse(4,6);
df{3}{2}{6} = a;

% df{3}{3}{1}
a = sparse(4,6);
df{3}{3}{1} = a;

% df{3}{3}{2}
a = sparse(4,6);
a(3,2) = (2*g*l1*lc2^2*m2^2*(l1^2*lc2^2*m2^2*cos(x1) + (l1^2*lc2^2*m2^2*cos(x1 - 2*x2))/2 + (l1^2*lc2^2*m2^2*cos(x1 + 2*x2))/2) - 2*I2*g*l1*lc2^2*m2^2*(2*I1*cos(x1 + 2*x2) + l1^2*m2*cos(x1 - 2*x2) + l1^2*m2*cos(x1 + 2*x2) + l1*lc1*m1*cos(x1 - 2*x2) - l1*lc1*m1*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(3,3) = (g*lc2*m2*cos(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) - I2*g*lc2*m2*cos(x1 + x2) + g*l1*lc2^2*m2^2*cos(x1 + x2)*cos(x2) - 2*g*l1*lc2^2*m2^2*sin(x1 + x2)*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) + (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(g*lc2*m2*sin(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) - I2*g*lc2*m2*sin(x1 + x2) + g*l1*lc2^2*m2^2*cos(x1 + x2)*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 + (2*(I2*(g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2)) - g*lc2*m2*cos(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 - ((I2*(g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2)) - g*lc2*m2*cos(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,2) = (((I2 + l1*lc2*m2*cos(x2))*(g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - g*lc2*m2*sin(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - g*lc2*m2*cos(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) + l1*lc2*m2*sin(x2)*(g*sin(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*sin(x1 + x2)) - 2*g*l1*lc2^2*m2^2*sin(x1 + x2)*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,3) = (g*lc2*m2*cos(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + l1*lc2*m2*cos(x2)*(g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2)) - 2*g*l1*lc2^2*m2^2*cos(x1 + x2)*cos(x2) + 2*g*l1*lc2^2*m2^2*sin(x1 + x2)*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (2*((g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 + (((g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2))*(I2 + l1*lc2*m2*cos(x2)) - g*lc2*m2*cos(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(g*lc2*m2*sin(x1 + x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) - g*lc2*m2*sin(x1 + x2)*(I2 + l1*lc2*m2*cos(x2)) - l1*lc2*m2*sin(x2)*(g*cos(x1)*(l1*m2 + lc1*m1) + g*lc2*m2*cos(x1 + x2)) + 2*g*l1*lc2^2*m2^2*cos(x1 + x2)*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
df{3}{3}{2} = a;

% df{3}{3}{3}
a = sparse(4,6);
a(3,2) = (2*g*l1*lc2^2*m2^2*(l1^2*lc2^2*m2^2*cos(x1 + 2*x2) - l1^2*lc2^2*m2^2*cos(x1 - 2*x2) - 4*I1*I2*cos(x1 + 2*x2) + 2*I2*l1^2*m2*cos(x1 - 2*x2) - 2*I2*l1^2*m2*cos(x1 + 2*x2) + 2*I2*l1*lc1*m1*cos(x1 - 2*x2) + 2*I2*l1*lc1*m1*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 + (4*g*l1^3*lc2^4*m2^4*sin(2*x2)*(4*I1*I2*sin(x1 + 2*x2) + 2*I2*l1^2*m2*sin(x1 - 2*x2) + 2*I2*l1^2*m2*sin(x1 + 2*x2) - 2*l1^2*lc2^2*m2^2*sin(x1) - l1^2*lc2^2*m2^2*sin(x1 - 2*x2) - l1^2*lc2^2*m2^2*sin(x1 + 2*x2) + 2*I2*l1*lc1*m1*sin(x1 - 2*x2) - 2*I2*l1*lc1*m1*sin(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
a(3,3) = 12*l1^3*lc2^3*m2^3*sin(2*x2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2*(b2*x4*cos(x2) - u1*cos(x2) + I2*x3^2*sin(x2) + I2*x4^2*sin(x2) + 2*I2*x3*x4*sin(x2) + 2*g*lc2*m2*sin(x1 + 2*x2) + 2*l1*lc2*m2*x3^2*sin(2*x2)) - 16*l1^6*lc2^6*m2^6*(3/4*sin(2*x2) - 1/4*sin(6*x2))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^4*(6*I2*b2*x4 - 6*I2*b1*x3 - 6*I2*u1 + 3*g*l1*lc2^2*m2^2*sin(x1) - 6*I2*g*l1*m2*sin(x1) - 6*I2*g*lc1*m1*sin(x1) + 3*l1^2*lc2^2*m2^2*x3^2*sin(2*x2) + 3*g*l1*lc2^2*m2^2*sin(x1 + 2*x2) - 6*l1*lc2*m2*u1*cos(x2) + 6*b2*l1*lc2*m2*x4*cos(x2) + 6*I2*l1*lc2*m2*x3^2*sin(x2) + 6*I2*l1*lc2*m2*x4^2*sin(x2) + 12*I2*l1*lc2*m2*x3*x4*sin(x2)) - 24*l1^3*lc2^3*m2^3*cos(2*x2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2*(u1*sin(x2) - b2*x4*sin(x2) + I2*x3^2*cos(x2) + I2*x4^2*cos(x2) + 2*I2*x3*x4*cos(x2) + g*lc2*m2*cos(x1 + 2*x2) + l1*lc2*m2*x3^2*cos(2*x2)) - 2*l1*lc2*m2/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))*(u1*sin(x2) - b2*x4*sin(x2) + I2*x3^2*cos(x2) + I2*x4^2*cos(x2) + 2*I2*x3*x4*cos(x2) + 4*g*lc2*m2*cos(x1 + 2*x2) + 4*l1*lc2*m2*x3^2*cos(2*x2)) + 16*l1^2*lc2^2*m2^2*sin(2*x2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2*(I2*b2*x4 - I2*b1*x3 - I2*u1 + 1/2*g*l1*lc2^2*m2^2*sin(x1) - I2*g*l1*m2*sin(x1) - I2*g*lc1*m1*sin(x1) + 1/2*l1^2*lc2^2*m2^2*x3^2*sin(2*x2) + 1/2*g*l1*lc2^2*m2^2*sin(x1 + 2*x2) - l1*lc2*m2*u1*cos(x2) + b2*l1*lc2*m2*x4*cos(x2) + I2*l1*lc2*m2*x3^2*sin(x2) + I2*l1*lc2*m2*x4^2*sin(x2) + 2*I2*l1*lc2*m2*x3*x4*sin(x2)) + 48*l1^4*lc2^4*m2^4*sin(4*x2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3*(I2*b2*x4 - I2*b1*x3 - I2*u1 + 1/2*g*l1*lc2^2*m2^2*sin(x1) - I2*g*l1*m2*sin(x1) - I2*g*lc1*m1*sin(x1) + 1/2*l1^2*lc2^2*m2^2*x3^2*sin(2*x2) + 1/2*g*l1*lc2^2*m2^2*sin(x1 + 2*x2) - l1*lc2*m2*u1*cos(x2) + b2*l1*lc2*m2*x4*cos(x2) + I2*l1*lc2*m2*x3^2*sin(x2) + I2*l1*lc2*m2*x4^2*sin(x2) + 2*I2*l1*lc2*m2*x3*x4*sin(x2)) - 48*l1^5*lc2^5*m2^5*(1/2*cos(4*x2) - 1/2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3*(u1*sin(x2) - b2*x4*sin(x2) + I2*x3^2*cos(x2) + I2*x4^2*cos(x2) + 2*I2*x3*x4*cos(x2) + g*lc2*m2*cos(x1 + 2*x2) + l1*lc2*m2*x3^2*cos(2*x2));
a(3,4) = - (4*l1*lc2*m2*(I2*x3*sin(x2) + I2*x4*sin(x2) + 2*l1*lc2*m2*x3*sin(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2)) - (8*l1^3*lc2^3*m2^3*sin(2*x2)*(I2*x3*cos(x2) + I2*x4*cos(x2) + l1*lc2*m2*x3*cos(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 - (8*l1^4*lc2^4*m2^4*(cos(4*x2) - 1)*(l1^2*lc2^2*m2^2*x3*sin(2*x2) - I2*b1 + 2*I2*l1*lc2*m2*x3*sin(x2) + 2*I2*l1*lc2*m2*x4*sin(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3 - (l1^2*lc2^2*m2^2*sin(2*x2)*(cos(x2)*(2*I2*l1*lc2*m2*x3 + 2*I2*l1*lc2*m2*x4) + 2*l1^2*lc2^2*m2^2*x3*cos(2*x2)))/((- cos(2*x2)/2 - 1/2)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2 - (8*l1^2*lc2^2*m2^2*cos(2*x2)*(l1^2*lc2^2*m2^2*x3*sin(2*x2) - I2*b1 + 2*I2*l1*lc2*m2*x3*sin(x2) + 2*I2*l1*lc2*m2*x4*sin(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(3,5) = -(8*l1*lc2*m2*(2*I1^2*I2^3*x3*sin(x2) + 2*I1^2*I2^3*x4*sin(x2) + I1^2*I2^2*b2*cos(x2) - (7*b2*l1^4*lc2^4*m2^4*cos(x2))/8 + 2*I2^3*l1^4*m2^2*x3*sin(x2) + 2*I2^3*l1^4*m2^2*x4*sin(x2) - (3*b2*l1^4*lc2^4*m2^4*cos(3*x2))/16 + (b2*l1^4*lc2^4*m2^4*cos(5*x2))/16 - (3*I2*b2*l1^3*lc2^3*m2^3)/2 + I2^2*b2*l1^4*m2^2*cos(x2) + 3*I2^2*l1^4*lc2^2*m2^3*x3*sin(3*x2) + 3*I2^2*l1^4*lc2^2*m2^3*x4*sin(3*x2) - (3*I2*b2*l1^4*lc2^2*m2^3*cos(x2))/2 - (11*I2*l1^4*lc2^4*m2^4*x3*sin(x2))/4 - (11*I2*l1^4*lc2^4*m2^4*x4*sin(x2))/4 + 2*I2^2*b2*l1^3*lc2*m2^2*cos(2*x2) - I2*b2*l1^3*lc2^3*m2^3*cos(2*x2) + (3*I2*b2*l1^4*lc2^2*m2^3*cos(3*x2))/2 + (I2*b2*l1^3*lc2^3*m2^3*cos(4*x2))/2 + 2*I1*I2^2*b2*l1^2*m2*cos(x2) - (21*I2*l1^4*lc2^4*m2^4*x3*sin(3*x2))/8 + (I2*l1^4*lc2^4*m2^4*x3*sin(5*x2))/8 - (21*I2*l1^4*lc2^4*m2^4*x4*sin(3*x2))/8 + (I2*l1^4*lc2^4*m2^4*x4*sin(5*x2))/8 - I2^2*l1^4*lc2^2*m2^3*x3*sin(x2) - I2^2*l1^4*lc2^2*m2^3*x4*sin(x2) + 4*I1*I2^3*l1^2*m2*x3*sin(x2) + 4*I1*I2^3*l1^2*m2*x4*sin(x2) - I1*I2^2*l1^2*lc2^2*m2^2*x3*sin(x2) - I1*I2^2*l1^2*lc2^2*m2^2*x4*sin(x2) + 2*I1*I2^2*b2*l1*lc2*m2*cos(2*x2) + 3*I1*I2^2*l1^2*lc2^2*m2^2*x3*sin(3*x2) + 3*I1*I2^2*l1^2*lc2^2*m2^2*x4*sin(3*x2) - (3*I1*I2*b2*l1^2*lc2^2*m2^2*cos(x2))/2 + (3*I1*I2*b2*l1^2*lc2^2*m2^2*cos(3*x2))/2))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
a(3,6) = (8*l1*lc2*m2*(I1^2*I2^2*cos(x2) - (3*I2*l1^3*lc2^3*m2^3)/2 + I2^2*l1^4*m2^2*cos(x2) - (7*l1^4*lc2^4*m2^4*cos(x2))/8 - (3*l1^4*lc2^4*m2^4*cos(3*x2))/16 + (l1^4*lc2^4*m2^4*cos(5*x2))/16 + 2*I2^2*l1^3*lc2*m2^2*cos(2*x2) - I2*l1^3*lc2^3*m2^3*cos(2*x2) + (3*I2*l1^4*lc2^2*m2^3*cos(3*x2))/2 + (I2*l1^3*lc2^3*m2^3*cos(4*x2))/2 + 2*I1*I2^2*l1^2*m2*cos(x2) - (3*I2*l1^4*lc2^2*m2^3*cos(x2))/2 - (3*I1*I2*l1^2*lc2^2*m2^2*cos(x2))/2 + (3*I1*I2*l1^2*lc2^2*m2^2*cos(3*x2))/2 + 2*I1*I2^2*l1*lc2*m2*cos(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
a(4,2) = (g*lc2*m2*(2*I1*cos(x1 + x2) + l1^2*m2*cos(x1 + x2) - l1^2*m2*cos(x1 - x2) - l1*lc1*m1*cos(x1 + x2) - l1*lc1*m1*cos(x1 - x2) + 4*l1*lc2*m2*cos(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2)) - (16*g*l1^4*lc2^4*m2^4*(cos(4*x2)/2 - 1/2)*(I2*l1*m2*cos(x1) - I1*lc2*m2*cos(x1 + x2) - l1^2*lc2*m2^2*cos(x1 + x2) + I2*lc1*m1*cos(x1) + l1^2*lc2*m2^2*cos(x1)*cos(x2) - l1*lc2^2*m2^2*cos(x1 + x2)*cos(x2) + l1*lc1*lc2*m1*m2*cos(x1)*cos(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3 - (8*g*l1^2*lc2^2*m2^2*cos(2*x2)*(I2*l1*m2*cos(x1) - I1*lc2*m2*cos(x1 + x2) - l1^2*lc2*m2^2*cos(x1 + x2) + I2*lc1*m1*cos(x1) + l1^2*lc2*m2^2*cos(x1)*cos(x2) - l1*lc2^2*m2^2*cos(x1 + x2)*cos(x2) + l1*lc1*lc2*m1*m2*cos(x1)*cos(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 - (4*g*l1^2*lc2^3*m2^3*sin(2*x2)*(2*I1*sin(x1 + x2) + l1^2*m2*sin(x1 + x2) + l1^2*m2*sin(x1 - x2) - l1*lc1*m1*sin(x1 + x2) + l1*lc1*m1*sin(x1 - x2) + 2*l1*lc2*m2*sin(x1 + 2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(4,3) = lc2*m2/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))*(4*l1*u1*sin(x2) + 2*I1*g*cos(x1 + x2) + g*l1^2*m2*cos(x1 - x2) + 2*l1^3*m2*x3^2*cos(x2) + 2*b1*l1*x3*sin(x2) - 4*b2*l1*x4*sin(x2) + g*l1^2*m2*cos(x1 + x2) + 2*I1*l1*x3^2*cos(x2) + 2*I2*l1*x3^2*cos(x2) + 2*I2*l1*x4^2*cos(x2) + 16*l1^2*lc2*m2*x3^2*cos(2*x2) + 8*l1^2*lc2*m2*x4^2*cos(2*x2) + g*l1*lc1*m1*cos(x1 - x2) + 8*g*l1*lc2*m2*cos(x1 + 2*x2) - g*l1*lc1*m1*cos(x1 + x2) + 4*I2*l1*x3*x4*cos(x2) + 16*l1^2*lc2*m2*x3*x4*cos(2*x2)) + 24*l1^4*lc2^5*m2^5*(1/2*cos(4*x2) - 1/2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3*(4*l1*u1*sin(x2) + 2*I1*g*cos(x1 + x2) + g*l1^2*m2*cos(x1 - x2) + 2*l1^3*m2*x3^2*cos(x2) + 2*b1*l1*x3*sin(x2) - 4*b2*l1*x4*sin(x2) + g*l1^2*m2*cos(x1 + x2) + 2*I1*l1*x3^2*cos(x2) + 2*I2*l1*x3^2*cos(x2) + 2*I2*l1*x4^2*cos(x2) + 4*l1^2*lc2*m2*x3^2*cos(2*x2) + 2*l1^2*lc2*m2*x4^2*cos(2*x2) + g*l1*lc1*m1*cos(x1 - x2) + 2*g*l1*lc2*m2*cos(x1 + 2*x2) - g*l1*lc1*m1*cos(x1 + x2) + 4*I2*l1*x3*x4*cos(x2) + 4*l1^2*lc2*m2*x3*x4*cos(2*x2)) - 6*l1^2*lc2^3*m2^3*sin(2*x2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2*(2*I1*g*sin(x1 + x2) - 4*l1*u1*cos(x2) - g*l1^2*m2*sin(x1 - x2) + 2*l1^3*m2*x3^2*sin(x2) - 2*b1*l1*x3*cos(x2) + 4*b2*l1*x4*cos(x2) + g*l1^2*m2*sin(x1 + x2) + 2*I1*l1*x3^2*sin(x2) + 2*I2*l1*x3^2*sin(x2) + 2*I2*l1*x4^2*sin(x2) + 8*l1^2*lc2*m2*x3^2*sin(2*x2) + 4*l1^2*lc2*m2*x4^2*sin(2*x2) - g*l1*lc1*m1*sin(x1 - x2) + 4*g*l1*lc2*m2*sin(x1 + 2*x2) - g*l1*lc1*m1*sin(x1 + x2) + 4*I2*l1*x3*x4*sin(x2) + 8*l1^2*lc2*m2*x3*x4*sin(2*x2)) - 16*l1^6*lc2^6*m2^6*(6*(I2 + l1*lc2*m2*cos(x2))*(- l1*lc2*m2*sin(x2)*x4^2 - 2*l1*lc2*m2*x3*sin(x2)*x4 + b1*x3 + g*lc2*m2*sin(x1 + x2) + g*l1*m2*sin(x1) + g*lc1*m1*sin(x1)) - 6*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))*(3/4*sin(2*x2) - 1/4*sin(6*x2))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^4 + 12*l1^2*lc2^3*m2^3*cos(2*x2)/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2*(4*l1*u1*sin(x2) + 2*I1*g*cos(x1 + x2) + g*l1^2*m2*cos(x1 - x2) + 2*l1^3*m2*x3^2*cos(x2) + 2*b1*l1*x3*sin(x2) - 4*b2*l1*x4*sin(x2) + g*l1^2*m2*cos(x1 + x2) + 2*I1*l1*x3^2*cos(x2) + 2*I2*l1*x3^2*cos(x2) + 2*I2*l1*x4^2*cos(x2) + 4*l1^2*lc2*m2*x3^2*cos(2*x2) + 2*l1^2*lc2*m2*x4^2*cos(2*x2) + g*l1*lc1*m1*cos(x1 - x2) + 2*g*l1*lc2*m2*cos(x1 + 2*x2) - g*l1*lc1*m1*cos(x1 + x2) + 4*I2*l1*x3*x4*cos(x2) + 4*l1^2*lc2*m2*x3*x4*cos(2*x2)) + 16*l1^2*lc2^2*m2^2*sin(2*x2)*((I2 + l1*lc2*m2*cos(x2))*(- l1*lc2*m2*sin(x2)*x4^2 - 2*l1*lc2*m2*x3*sin(x2)*x4 + b1*x3 + g*lc2*m2*sin(x1 + x2) + g*l1*m2*sin(x1) + g*lc1*m1*sin(x1)) - (m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 + 48*l1^4*lc2^4*m2^4*sin(4*x2)*((I2 + l1*lc2*m2*cos(x2))*(- l1*lc2*m2*sin(x2)*x4^2 - 2*l1*lc2*m2*x3*sin(x2)*x4 + b1*x3 + g*lc2*m2*sin(x1 + x2) + g*l1*m2*sin(x1) + g*lc1*m1*sin(x1)) - (m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)*(l1*lc2*m2*sin(x2)*x3^2 - u1 + b2*x4 + g*lc2*m2*sin(x1 + x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
a(4,4) = (2*l1*lc2*m2*(2*I1*x3*sin(x2) - b1*cos(x2) + 2*I2*x3*sin(x2) + 2*I2*x4*sin(x2) + 2*l1^2*m2*x3*sin(x2) + 8*l1*lc2*m2*x3*sin(2*x2) + 4*l1*lc2*m2*x4*sin(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2)) + (8*l1^3*lc2^3*m2^3*sin(2*x2)*(b1*sin(x2) + 2*I1*x3*cos(x2) + 2*I2*x3*cos(x2) + 2*I2*x4*cos(x2) + 2*l1^2*m2*x3*cos(x2) + 4*l1*lc2*m2*x3*cos(2*x2) + 2*l1*lc2*m2*x4*cos(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 - (8*l1^2*lc2^2*m2^2*cos(2*x2)*((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 - (8*l1^4*lc2^4*m2^4*(2*(I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 4*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(cos(4*x2)/2 - 1/2))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
a(4,5) = (4*l1*lc2*m2*(b2*cos(x2) + I2*x3*sin(x2) + I2*x4*sin(x2) + 2*l1*lc2*m2*x3*sin(2*x2) + 2*l1*lc2*m2*x4*sin(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2)) + (8*l1^4*lc2^4*m2^4*(cos(4*x2)/2 - 1/2)*(2*I1*b2 + 2*I2*b2 + 2*b2*l1^2*m2 + 2*l1^2*lc2^2*m2^2*x3*sin(2*x2) + 2*l1^2*lc2^2*m2^2*x4*sin(2*x2) + 4*b2*l1*lc2*m2*cos(x2) + 4*I2*l1*lc2*m2*x3*sin(x2) + 4*I2*l1*lc2*m2*x4*sin(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3 + (16*l1^3*lc2^3*m2^3*sin(2*x2)*(I2*x3*cos(x2) - b2*sin(x2) + I2*x4*cos(x2) + l1*lc2*m2*x3*cos(2*x2) + l1*lc2*m2*x4*cos(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2 + (8*l1^2*lc2^2*m2^2*cos(2*x2)*(I1*b2 + I2*b2 + b2*l1^2*m2 + l1^2*lc2^2*m2^2*x3*sin(2*x2) + l1^2*lc2^2*m2^2*x4*sin(2*x2) + 2*b2*l1*lc2*m2*cos(x2) + 2*I2*l1*lc2*m2*x3*sin(x2) + 2*I2*l1*lc2*m2*x4*sin(x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(4,6) = -(16*l1*lc2*m2*(I1^2*I2^2*cos(x2) - (3*l1^5*lc2^3*m2^4)/4 - (3*I1*l1^3*lc2^3*m2^3)/4 - (3*I2*l1^3*lc2^3*m2^3)/4 + I2^2*l1^4*m2^2*cos(x2) - (7*l1^4*lc2^4*m2^4*cos(x2))/8 - (l1^5*lc2^3*m2^4*cos(2*x2))/2 - (3*l1^4*lc2^4*m2^4*cos(3*x2))/16 + (l1^5*lc2^3*m2^4*cos(4*x2))/4 + (l1^4*lc2^4*m2^4*cos(5*x2))/16 - (I1*l1^3*lc2^3*m2^3*cos(2*x2))/2 + (I1*l1^3*lc2^3*m2^3*cos(4*x2))/4 + I2^2*l1^3*lc2*m2^2*cos(2*x2) - (I2*l1^3*lc2^3*m2^3*cos(2*x2))/2 + (3*I2*l1^4*lc2^2*m2^3*cos(3*x2))/2 + (I2*l1^3*lc2^3*m2^3*cos(4*x2))/4 + 2*I1*I2^2*l1^2*m2*cos(x2) + I2*l1^5*lc2*m2^3*cos(2*x2) - (3*I2*l1^4*lc2^2*m2^3*cos(x2))/2 + 2*I1*I2*l1^3*lc2*m2^2*cos(2*x2) - (3*I1*I2*l1^2*lc2^2*m2^2*cos(x2))/2 + (3*I1*I2*l1^2*lc2^2*m2^2*cos(3*x2))/2 + I1*I2^2*l1*lc2*m2*cos(2*x2) + I1^2*I2*l1*lc2*m2*cos(2*x2)))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^3;
df{3}{3}{3} = a;

% df{3}{3}{4}
a = sparse(4,6);
a(3,3) = (6*l1^2*lc2^2*m2^2*x3*cos(x2)*sin(x2) + 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*x4*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - ((I2*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(2*l1*lc2*m2*x3*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*I2*l1*lc2*m2*x4*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 + (2*(I2*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3;
a(3,4) = - (2*l1^2*lc2^2*m2^2*(2*sin(x2)^2 - 1) + 2*I2*l1*lc2*m2*(2*sin(x2/2)^2 - 1))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2) - (l1^2*lc2^2*m2^2*sin(2*x2)*(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2;
a(3,5) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = (((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (12*l1^2*lc2^2*m2^2*x3*cos(x2)*sin(x2) - l1*lc2*m2*cos(x2)*(b1 - 2*l1*lc2*m2*x4*sin(x2)) + 4*l1^2*lc2^2*m2^2*x4*cos(x2)*sin(x2) + 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*x4*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (2*((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 + (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(l1*lc2*m2*sin(x2)*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 4*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*l1*lc2*m2*x3*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*x4*cos(x2)*(I2 + l1*lc2*m2*cos(x2))))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,4) = (2*l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (4*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,5) = -(2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
df{3}{3}{4} = a;

% df{3}{3}{5}
a = sparse(4,6);
a(3,3) = (I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*l1*lc2*m2*cos(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (2*(I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 - (2*(I2*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) - b2*l1*lc2*m2*sin(x2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 + ((I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*(I2 + l1*lc2*m2*cos(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(3,4) = (lc2*(8*cos(x2)*I2^2*l1^3*m2^2 + 8*I1*cos(x2)*I2^2*l1*m2) - 8*I2*l1^3*lc2^3*m2*((5*m2^2*cos(x2))/4 - (m2^2*cos(3*x2))/4))/(2*I1*I2 - l1^2*lc2^2*m2^2 + 2*I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(2*x2))^2;
a(3,5) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = (2*(b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 - ((b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) - (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) + 2*b2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - ((I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + 2*l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) + 2*b2*l1*lc2*m2*cos(x2) + l1*lc2*m2*cos(x2)*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,4) = -(2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
a(4,5) = -(2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
df{3}{3}{5} = a;

% df{3}{3}{6}
a = sparse(4,6);
a(3,3) = -(I2*(m2^4*(l1*(l1^3*cos(2*x2) - (l1^3*cos(4*x2))/2 + (3*l1^3)/2)*lc2^4 + l1*((3*l1^4*cos(x2))/2 - (3*l1^4*cos(3*x2))/2)*lc2^3) + l1*lc2^3*m2^3*((3*I1*l1^2*cos(x2))/2 - (3*I1*l1^2*cos(3*x2))/2)) - I2^2*(m2^3*(cos(x2)*l1^5*lc2 + 2*cos(2*x2)*l1^4*lc2^2) + m2^2*(2*I1*cos(x2)*l1^3*lc2 + 2*I1*cos(2*x2)*l1^2*lc2^2) + I1^2*l1*lc2*m2*cos(x2)) + l1*lc2^5*m2^5*((7*l1^4*cos(x2))/8 + (3*l1^4*cos(3*x2))/16 - (l1^4*cos(5*x2))/16))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^3;
a(4,3) = -(2*l1*lc2*m2*(I1^2*I2^2*cos(x2) - (3*l1^5*lc2^3*m2^4)/4 - (3*I1*l1^3*lc2^3*m2^3)/4 - (3*I2*l1^3*lc2^3*m2^3)/4 + I2^2*l1^4*m2^2*cos(x2) - (7*l1^4*lc2^4*m2^4*cos(x2))/8 - (3*l1^4*lc2^4*m2^4*cos(3*x2))/16 + (l1^5*lc2^3*m2^4*cos(4*x2))/4 + (l1^4*lc2^4*m2^4*cos(5*x2))/16 + (I1*l1^3*lc2^3*m2^3*cos(4*x2))/4 + (3*I2*l1^4*lc2^2*m2^3*cos(3*x2))/2 + (I2*l1^3*lc2^3*m2^3*cos(4*x2))/4 + 2*I1*I2^2*l1^2*m2*cos(x2) - (3*I2*l1^4*lc2^2*m2^3*cos(x2))/2 - (3*I1*I2*l1^2*lc2^2*m2^2*cos(x2))/2 + (3*I1*I2*l1^2*lc2^2*m2^2*cos(3*x2))/2) + 2*l1*lc2*m2*cos(2*x2)*(I1^2*I2*l1*lc2*m2 + I1*I2^2*l1*lc2*m2 + 2*I1*I2*l1^3*lc2*m2^2 - (I1*l1^3*lc2^3*m2^3)/2 + I2^2*l1^3*lc2*m2^2 + I2*l1^5*lc2*m2^3 - (I2*l1^3*lc2^3*m2^3)/2 - (l1^5*lc2^3*m2^4)/2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^3;
df{3}{3}{6} = a;

% df{3}{4}{1}
a = sparse(4,6);
df{3}{4}{1} = a;

% df{3}{4}{2}
a = sparse(4,6);
df{3}{4}{2} = a;

% df{3}{4}{3}
a = sparse(4,6);
a(3,3) = (6*l1^2*lc2^2*m2^2*x3*cos(x2)*sin(x2) + 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*x4*sin(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - ((I2*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(2*l1*lc2*m2*x3*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*I2*l1*lc2*m2*x4*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 + (2*(I2*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3;
a(3,4) = (2*cos(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*cos(x2)*l1*lc2*m2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2) - (2*l1^2*lc2^2*m2^2*cos(x2)*sin(x2)*(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2;
a(3,5) = (2*I2*l1*lc2*m2*cos(x2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2) - (4*I2*l1^3*lc2^3*m2^3*cos(x2)*sin(x2)^2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2;
a(4,3) = (((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (12*l1^2*lc2^2*m2^2*x3*cos(x2)*sin(x2) - l1*lc2*m2*cos(x2)*(b1 - 2*l1*lc2*m2*x4*sin(x2)) + 4*l1^2*lc2^2*m2^2*x4*cos(x2)*sin(x2) + 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*x4*sin(x2)*(I2 + l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (2*((I2 + l1*lc2*m2*cos(x2))*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 2*l1*lc2*m2*x3*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 + (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(l1*lc2*m2*sin(x2)*(b1 - 2*l1*lc2*m2*x4*sin(x2)) - 4*l1^2*lc2^2*m2^2*x3*sin(x2)^2 + 2*l1*lc2*m2*x3*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + 2*l1*lc2*m2*x4*cos(x2)*(I2 + l1*lc2*m2*cos(x2))))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(4,4) = (2*l1^2*lc2^2*m2^2*cos(x2)*sin(x2)*(2*l1^2*lc2^2*m2^2*sin(2*x2) + 2*l1*lc2*m2*sin(x2)*(m2*l1^2 + I1 + I2)))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2 - (4*l1^2*lc2^2*m2^2*cos(2*x2) + 2*l1*lc2*m2*cos(x2)*(m2*l1^2 + I1 + I2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
a(4,5) = (2*l1^2*lc2^2*m2^2*cos(x2)*sin(x2)*(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2 - (2*cos(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*cos(x2)*l1*lc2*m2)/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2);
df{3}{4}{3} = a;

% df{3}{4}{4}
a = sparse(4,6);
a(3,3) = (2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
a(4,3) = (2*l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (4*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
df{3}{4}{4} = a;

% df{3}{4}{5}
a = sparse(4,6);
a(3,3) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = -(2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
df{3}{4}{5} = a;

% df{3}{4}{6}
a = sparse(4,6);
df{3}{4}{6} = a;

% df{3}{5}{1}
a = sparse(4,6);
df{3}{5}{1} = a;

% df{3}{5}{2}
a = sparse(4,6);
df{3}{5}{2} = a;

% df{3}{5}{3}
a = sparse(4,6);
a(3,3) = (I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*l1*lc2*m2*cos(x2))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2)) - (2*(I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*(I2 + l1*lc2*m2*cos(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 - (2*(I2*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) - b2*l1*lc2*m2*sin(x2))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 + ((I2*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + b2*(I2 + l1*lc2*m2*cos(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2;
a(3,4) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(3,5) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = (2*(b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))^2)/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^3 - ((b2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2) + (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))*(2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2)) + 2*I2*l1*lc2*m2*cos(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - (2*(2*l1*lc2*m2*sin(x2)*(I2 + l1*lc2*m2*cos(x2)) - 2*I2*l1*lc2*m2*sin(x2))*(l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) - (I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) + 2*b2*l1*lc2*m2*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2))^2 - ((I2 + l1*lc2*m2*cos(x2))*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)) + 2*l1*lc2*m2*sin(x2)*(2*l1*lc2*m2*x3*cos(x2) + 2*l1*lc2*m2*x4*cos(x2)) + 2*b2*l1*lc2*m2*cos(x2) + l1*lc2*m2*cos(x2)*(2*l1*lc2*m2*x3*sin(x2) + 2*l1*lc2*m2*x4*sin(x2)))/((I2 + l1*lc2*m2*cos(x2))^2 - I2*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1 + I2));
a(4,4) = (2*l1^2*lc2^2*m2^2*(2*sin(x2)^2 - 1) + 2*I2*l1*lc2*m2*(2*sin(x2/2)^2 - 1))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2) + (l1^2*lc2^2*m2^2*sin(2*x2)*(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2;
a(4,5) = (2*l1^2*lc2^2*m2^2*(2*sin(x2)^2 - 1) + 2*I2*l1*lc2*m2*(2*sin(x2/2)^2 - 1))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2) + (l1^2*lc2^2*m2^2*sin(2*x2)*(sin(2*x2)*l1^2*lc2^2*m2^2 + 2*I2*sin(x2)*l1*lc2*m2))/((sin(x2)^2 - 1)*l1^2*lc2^2*m2^2 + I2*l1^2*m2 + I1*I2)^2;
df{3}{5}{3} = a;

% df{3}{5}{4}
a = sparse(4,6);
a(3,3) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = -(2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
df{3}{5}{4} = a;

% df{3}{5}{5}
a = sparse(4,6);
a(3,3) = (2*I2*l1^3*lc2^3*m2^3*cos(x2)^3 + 2*I2*l1*lc2*m2*cos(x2)*(I2*l1^2*m2 - 2*l1^2*lc2^2*m2^2 + I1*I2))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2;
a(4,3) = -(2*l1*lc2*m2*(I1*I2^2*cos(x2) - (l1^3*lc2^3*m2^3)/2 + I2^2*l1^2*m2*cos(x2) + (I2*l1^2*lc2^2*m2^2*cos(3*x2))/4 - (5*I2*l1^2*lc2^2*m2^2*cos(x2))/4) + 2*l1*lc2*m2*cos(2*x2)*(I2*l1^3*lc2*m2^2 - (l1^3*lc2^3*m2^3)/2 + I1*I2*l1*lc2*m2))/(I1*I2 - (l1^2*lc2^2*m2^2)/2 + I2*l1^2*m2 - (l1^2*lc2^2*m2^2*cos(2*x2))/2)^2;
df{3}{5}{5} = a;

% df{3}{5}{6}
a = sparse(4,6);
df{3}{5}{6} = a;

% df{3}{6}{1}
a = sparse(4,6);
df{3}{6}{1} = a;

% df{3}{6}{2}
a = sparse(4,6);
df{3}{6}{2} = a;

% df{3}{6}{3}
a = sparse(4,6);
a(3,3) = (l1^3*lc2^3*m2^3*cos(x2)^3 - 2*I2*l1^2*lc2^2*m2^2*sin(x2)^2 - 2*l1^3*lc2^3*m2^3*cos(x2)*sin(x2)^2 + I2*l1*lc2*m2*cos(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2 - (4*l1^2*lc2^2*m2^2*cos(x2)*sin(x2)*(l1^3*lc2^3*m2^3*cos(x2)^2*sin(x2) + I2*l1*lc2*m2*sin(x2)*(m2*l1^2 + 2*lc2*m2*cos(x2)*l1 + I1)))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^3;
a(4,3) = (2*l1^2*lc2^2*m2^2*sin(x2)^2*(m2*l1^2 + lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2 + (2*l1^2*lc2^2*m2^2*sin(x2)^2*(I2 + l1*lc2*m2*cos(x2)))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2 - (2*l1*lc2*m2*cos(x2)*(I2 + l1*lc2*m2*cos(x2))*(m2*l1^2 + lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^2 + (8*l1^3*lc2^3*m2^3*cos(x2)*sin(x2)^2*(I2 + l1*lc2*m2*cos(x2))*(m2*l1^2 + lc2*m2*cos(x2)*l1 + I1))/(I2*l1^2*m2 - l1^2*lc2^2*m2^2*cos(x2)^2 + I1*I2)^3;
df{3}{6}{3} = a;

% df{3}{6}{4}
a = sparse(4,6);
df{3}{6}{4} = a;

% df{3}{6}{5}
a = sparse(4,6);
df{3}{6}{5} = a;

% df{3}{6}{6}
a = sparse(4,6);
df{3}{6}{6} = a;

end  % if (order>=3)
