function df = cartpole_gradients(r,t,x,u,order)

% This file evaluates the gradients of the robot dynamics.
% Usage:  df = cartpole_gradients(r,t,x,u,order)
%   r 		- cartpole class object
%   t 		- time (scalar)
%   x 		- 4-by-1 state vector
%   u 		- 1-by-1 input vector
%   order 	- order of gradients to compute (1<=order<=3)
% This is an auto-generated file.  See <a href="matlab: help robot>generate_dynamics_func_gradients">robot>generate_dynamics_func_gradients</a>. 

% NOTEST

% Check inputs:
if (~isa(r,'CartPoleDynamics')) error(' r must be a CartPoleDynamics class object'); end
if (length(t)~=1) error('t must be a scalar'); end
if (any(size(x)~=[4,1])) error('x must be a 4-by-1 vector'); end
if (any(size(u)~=[1,1])) error('u must be a 1-by-1 vector'); end
if ((order<1) || (order>3)) error('1<=order<=3'); end

% Symbol table:
mc=r.mc; mp=r.mp; l=r.l; g=r.g;   
x1=x(1);x2=x(2);x3=x(3);x4=x(4);
u1=u(1);

% df{1}
a = sparse(4,6);
a(1,4) = 1;
a(2,5) = 1;
a(3,3) = (mp*cos(x2)*(l*x4^2 + g*cos(x2)) - g*mp*sin(x2)^2)/(mp*sin(x2)^2 + mc) - (2*mp*cos(x2)*sin(x2)*(u1 + mp*sin(x2)*(l*x4^2 + g*cos(x2))))/(mp*sin(x2)^2 + mc)^2;
a(3,5) = (2*l*mp*x4*sin(x2))/(mp*sin(x2)^2 + mc);
a(3,6) = 1/(mp*sin(x2)^2 + mc);
a(4,3) = (l*mp*x4^2*sin(x2)^2 - l*mp*x4^2*cos(x2)^2 - g*(mc + mp)*cos(x2) + u1*sin(x2))/(l*(mp*sin(x2)^2 + mc)) + (2*mp*cos(x2)*sin(x2)*(l*mp*cos(x2)*sin(x2)*x4^2 + u1*cos(x2) + g*sin(x2)*(mc + mp)))/(l*(mp*sin(x2)^2 + mc)^2);
a(4,5) = -(mp*x4*sin(2*x2))/(mp*sin(x2)^2 + mc);
a(4,6) = -cos(x2)/(l*(mc - mp*(cos(x2)^2 - 1)));
df{1} = a;


if (order>=3)
% df{2}{1}
a = sparse(4,6);
df{2}{1} = a;

% df{2}{2}
a = sparse(4,6);
df{2}{2} = a;

% df{2}{3}
a = sparse(4,6);
a(3,3) = (sin(x2)^3*(mp*(2*l*mp^2*x4^2*cos(x2)^2 + 2*g*mp^2*cos(x2)^3) - 2*g*mc*mp^2*cos(x2)) - sin(x2)*(mp*(l*x4^2 + 4*g*cos(x2))*mc^2 + mp*(6*l*mp*x4^2*cos(x2)^2 + 6*g*mp*cos(x2)^3)*mc) + sin(x2)^2*(6*u1*mp^2*cos(x2)^2 + 2*mc*u1*mp) + mp*sin(x2)^5*(l*mp^2*x4^2 + 2*g*cos(x2)*mp^2) + 2*mp^2*u1*sin(x2)^4 - 2*mc*mp*u1*cos(x2)^2)/(mp*sin(x2)^2 + mc)^3;
a(3,5) = (2*l*mp*x4*cos(x2)*(mp*cos(x2)^2 + mc - mp))/(mc - mp*cos(x2)^2 + mp)^2;
a(3,6) = -(mp*sin(2*x2))/(mp*sin(x2)^2 + mc)^2;
a(4,3) = (2*mp^3*x4^2*sin(4*x2) - 4*mp^3*x4^2*sin(2*x2) + 16*mc*mp^2*x4^2*sin(2*x2) + 16*mc^2*mp*x4^2*sin(2*x2) + 4*mc*mp^2*x4^2*sin(4*x2))/(2*mc + mp - mp*cos(2*x2))^3 + (8*mc^2*u1*cos(x2) - 11*mp^2*u1*cos(x2) + 8*g*mc^3*sin(x2) - 7*g*mp^3*sin(x2) + (21*mp^2*u1*cos(3*x2))/2 + (mp^2*u1*cos(5*x2))/2 + (3*g*mp^3*sin(3*x2))/2 + (g*mp^3*sin(5*x2))/2 + (27*g*mc*mp^2*sin(3*x2))/2 + 12*g*mc^2*mp*sin(3*x2) + (g*mc*mp^2*sin(5*x2))/2 + 4*mc*mp*u1*cos(x2) + 12*mc*mp*u1*cos(3*x2) + 5*g*mc*mp^2*sin(x2) + 20*g*mc^2*mp*sin(x2))/(l*(2*mc + mp - mp*cos(2*x2))^3);
a(4,5) = (4*mp^2*x4 - 8*mp*x4*cos(2*x2)*(mc + mp/2))/(2*mc + mp - mp*cos(2*x2))^2;
a(4,6) = -(mp*sin(x2)^3 - sin(x2)*(mc + 2*mp))/(l*(mp*sin(x2)^2 + mc)^2);
df{2}{3} = a;

% df{2}{4}
a = sparse(4,6);
df{2}{4} = a;

% df{2}{5}
a = sparse(4,6);
a(3,3) = (2*l*mp*x4*cos(x2)*(mp*cos(x2)^2 + mc - mp))/(mc - mp*cos(x2)^2 + mp)^2;
a(3,5) = (2*l*mp*sin(x2))/(mp*sin(x2)^2 + mc);
a(4,3) = -(2*mc*mp*x4 - 2*mp*x4*sin(x2)^2*(2*mc + mp))/(mp*sin(x2)^2 + mc)^2;
a(4,5) = -(mp*sin(2*x2))/(mp*sin(x2)^2 + mc);
df{2}{5} = a;

% df{2}{6}
a = sparse(4,6);
a(3,3) = -(mp*sin(2*x2))/(mp*sin(x2)^2 + mc)^2;
a(4,3) = -(mp*sin(x2)^3 - sin(x2)*(mc + 2*mp))/(l*(mp*sin(x2)^2 + mc)^2);
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
df{3}{2}{2} = a;

% df{3}{2}{3}
a = sparse(4,6);
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
df{3}{3}{2} = a;

% df{3}{3}{3}
a = sparse(4,6);
a(3,3) = -(mp*cos(x2)^5*(3*l*mp^3*x4^2 + 23*l*mc*mp^2*x4^2 - 8*u1*sin(x2)*mp^2) - mp*(4*g*mc^3 + 6*g*mc^2*mp - 2*g*mp^3) + mp*cos(x2)*(l*mc^3*x4^2 - 17*l*mc^2*mp*x4^2 - 8*u1*sin(x2)*mc^2 - 13*l*mc*mp^2*x4^2 + 8*u1*sin(x2)*mc*mp + 5*l*mp^3*x4^2 + 16*u1*sin(x2)*mp^2) - mp*cos(x2)^3*(10*l*mc*mp^2*x4^2 - 23*l*mc^2*mp*x4^2 + 32*u1*sin(x2)*mc*mp + 9*l*mp^3*x4^2 + 8*u1*sin(x2)*mp^2) + mp*cos(x2)^6*(4*g*mp^3 + 8*g*mc*mp^2) - mp*cos(x2)^2*(20*g*mc^2*mp - 8*g*mc^3 + 28*g*mc*mp^2) + mp*cos(x2)^4*(32*g*mc^2*mp + 20*g*mc*mp^2 - 6*g*mp^3) + l*mp^4*x4^2*cos(x2)^7)/(mc - mp*cos(x2)^2 + mp)^4;
a(3,5) = -(2*l*mp*x4*sin(x2)*(mp^2*(sin(x2)^2 - 1)^2 + mc^2 - mp^2 - 6*mc*mp*(sin(x2)^2 - 1)))/(mc + mp + mp*(sin(x2)^2 - 1))^3;
a(3,6) = -(mp^2*(4*cos(4*x2) - 12) + cos(2*x2)*(8*mp^2 + 16*mc*mp))/(2*mc + mp - mp*cos(2*x2))^3;
a(4,3) = (mp^4*x4^2 - cos(2*x2)*((mc*mp^3*x4^2)/4 - 6*mc^2*mp^2*x4^2 - 4*mc^3*mp*x4^2 + (9*mp^4*x4^2)/8) + (mp^4*x4^2*cos(6*x2))/8 - 4*mc*mp^3*x4^2 - 4*mc^2*mp^2*x4^2 + 4*mc^2*mp^2*x4^2*cos(4*x2) + 4*mc*mp^3*x4^2*cos(4*x2) + (mc*mp^3*x4^2*cos(6*x2))/4)/(mc + mp/2 - (mp*cos(2*x2))/2)^4 + (g*mc^4*cos(x2) + (43*g*mp^4*cos(x2))/64 - mc^3*u1*sin(x2) + (387*mp^3*u1*sin(x2))/64 - (63*g*mp^4*cos(3*x2))/64 + (19*g*mp^4*cos(5*x2))/64 + (g*mp^4*cos(7*x2))/64 - (5*mp^3*u1*sin(3*x2))/64 - (73*mp^3*u1*sin(5*x2))/64 - (mp^3*u1*sin(7*x2))/64 + (237*g*mc*mp^3*cos(3*x2))/64 + (23*g*mc^3*mp*cos(3*x2))/4 + (111*g*mc*mp^3*cos(5*x2))/64 + (g*mc*mp^3*cos(7*x2))/64 - (47*g*mc^2*mp^2*cos(x2))/8 - (109*mc*mp^2*u1*sin(3*x2))/16 - (23*mc^2*mp*u1*sin(3*x2))/4 - (23*mc*mp^2*u1*sin(5*x2))/16 + (167*g*mc^2*mp^2*cos(3*x2))/16 + (23*g*mc^2*mp^2*cos(5*x2))/16 - (349*g*mc*mp^3*cos(x2))/64 + (5*g*mc^3*mp*cos(x2))/4 + (29*mc*mp^2*u1*sin(x2))/8 - (11*mc^2*mp*u1*sin(x2))/4)/(l*(mc + mp/2 - (mp*cos(2*x2))/2)^4);
a(4,5) = (8*mp*x4*sin(2*x2)*(4*mc^2 + 4*mc*mp - mp^2) + 8*mp*x4*cos(2*x2)*sin(2*x2)*(mp^2 + 2*mc*mp))/(2*mc + mp - mp*cos(2*x2))^3;
a(4,6) = (cos(x2)^3*(4*mp^2 + 6*mc*mp) - cos(x2)*(4*mc*mp - mc^2 + 5*mp^2) + mp^2*cos(x2)^5)/(l*(mc - mp*(cos(x2)^2 - 1))^3);
df{3}{3}{3} = a;

% df{3}{3}{4}
a = sparse(4,6);
df{3}{3}{4} = a;

% df{3}{3}{5}
a = sparse(4,6);
a(3,3) = -(2*sin(x2)*l*mp*x4*(mc^2 + 6*mc*mp*cos(x2)^2 - 2*mp^2*cos(x2)^2*sin(x2)^2 - mp^2*sin(x2)^4))/(mp*sin(x2)^2 + mc)^3;
a(3,5) = (2*l*mp*cos(x2)*(mp*cos(x2)^2 + mc - mp))/(- mp*cos(x2)^2 + mc + mp)^2;
a(4,3) = (4*mp*x4*(8*mc^2*sin(2*x2) - 2*mp^2*sin(2*x2) + mp^2*sin(4*x2) + 8*mc*mp*sin(2*x2) + 2*mc*mp*sin(4*x2)))/(2*mc + mp - mp*cos(2*x2))^3;
a(4,5) = -(4*mp*(2*mc*cos(2*x2) - mp + mp*cos(2*x2)))/(2*mc + mp - mp*cos(2*x2))^2;
df{3}{3}{5} = a;

% df{3}{3}{6}
a = sparse(4,6);
a(3,3) = ((6*sin(x2)^2 - 4*sin(x2)^4)*mp^2 + mc*(4*sin(x2)^2 - 2)*mp)/(mp*sin(x2)^2 + mc)^3;
a(4,3) = (8*cos(x2)*mc^2 + (12*cos(3*x2) + 4*cos(x2))*mc*mp + ((21*cos(3*x2))/2 + cos(5*x2)/2 - 11*cos(x2))*mp^2)/(l*(2*mc + mp - mp*cos(2*x2))^3);
df{3}{3}{6} = a;

% df{3}{4}{1}
a = sparse(4,6);
df{3}{4}{1} = a;

% df{3}{4}{2}
a = sparse(4,6);
df{3}{4}{2} = a;

% df{3}{4}{3}
a = sparse(4,6);
df{3}{4}{3} = a;

% df{3}{4}{4}
a = sparse(4,6);
df{3}{4}{4} = a;

% df{3}{4}{5}
a = sparse(4,6);
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
a(3,3) = -(2*sin(x2)*l*mp*x4*(mc^2 + 6*mc*mp*cos(x2)^2 + mp^2*cos(x2)^4 - mp^2))/(- mp*cos(x2)^2 + mc + mp)^3;
a(3,5) = (2*cos(x2)*l*mp*(mc - mp*sin(x2)^2))/(mp*sin(x2)^2 + mc)^2;
a(4,3) = (4*sin(x2)*cos(x2)*mp*x4*(2*mc^2 - 2*mc*mp*sin(x2)^2 + 3*mc*mp - mp^2*sin(x2)^2))/(mp*sin(x2)^2 + mc)^3;
a(4,5) = -(2*mp*(mp*cos(2*x2)*sin(x2)^2 - mp*sin(2*x2)*cos(x2)*sin(x2) + mc*cos(2*x2)))/(mp*sin(x2)^2 + mc)^2;
df{3}{5}{3} = a;

% df{3}{5}{4}
a = sparse(4,6);
df{3}{5}{4} = a;

% df{3}{5}{5}
a = sparse(4,6);
a(3,3) = (2*l*mp*cos(x2)*(mp*cos(x2)^2 + mc - mp))/(mc - mp*cos(x2)^2 + mp)^2;
a(4,3) = -(2*mc*mp - 2*mp*sin(x2)^2*(2*mc + mp))/(mp*sin(x2)^2 + mc)^2;
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
a(3,3) = -(mp^2*(4*cos(4*x2) - 12) + cos(2*x2)*(8*mp^2 + 16*mc*mp))/(2*mc + mp - mp*cos(2*x2))^3;
a(4,3) = (cos(x2)^3*(4*mp^2 + 6*mc*mp) - cos(x2)*(4*mc*mp - mc^2 + 5*mp^2) + mp^2*cos(x2)^5)/(l*(mc - mp*(cos(x2)^2 - 1))^3);
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
