function [df, d2f] = dynamicsGradients_w(a1, a2, a3, a4, a5, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'CartPolePlant');
if (nargin<5) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[4  1]);
sizecheck(a4,[1  1]);
sizecheck(a5,[1  1]);

% Symbol table:
a3_2=a3(2);
a3_3=a3(3);
a3_4=a3(4);
a4_1=a4(1);
a5_1=a5(1);
g=a1.g;
l=a1.l;
mc=a1.mc;
mp=a1.mp;


% Compute Gradients:
df = sparse(4,7);
df(1,4) = 1;
df(2,5) = 1;
df(3,3) = (mp*(2*g*cos(a3_2)^2 - g + a3_4^2*l*cos(a3_2)))/(mc + mp - mp*cos(a3_2)^2) - (2*mp*cos(a3_2)*sin(a3_2)*(a4_1 + a5_1 + g*mp*cos(a3_2)*sin(a3_2) + a3_4^2*l*mp*sin(a3_2)))/(mc + mp - mp*cos(a3_2)^2)^2;
df(3,5) = (2*a3_4*l*mp*sin(a3_2))/(mc + mp*sin(a3_2)^2);
df(3,6) = 1/(mc + mp - mp*cos(a3_2)^2);
df(3,7) = 1/(mc + mp - mp*cos(a3_2)^2);
df(4,3) = (2*mp*cos(a3_2)*sin(a3_2)*(a4_1*cos(a3_2) + a5_1*cos(a3_2) + g*mc*sin(a3_2) + g*mp*sin(a3_2) + a3_4^2*l*mp*cos(a3_2)*sin(a3_2)))/(l*(mc + mp - mp*cos(a3_2)^2)^2) - (2*(g*mc*cos(a3_2) - a5_1*sin(a3_2) - a4_1*sin(a3_2) + g*mp*cos(a3_2) + a3_4^2*l*mp*(2*cos(a3_2)^2 - 1)))/(l*(2*mc + mp - mp*(2*cos(a3_2)^2 - 1)));
df(4,5) = -(2*a3_4*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2));
df(4,6) = -cos(a3_2)/(l*(mc + mp - mp*cos(a3_2)^2));
df(4,7) = -cos(a3_2)/(l*(mc + mp - mp*cos(a3_2)^2));

% d2f
if (order>=2)
  d2f = sparse(4,49);
  d2f(3,17) = (2*mp*sin(a3_2)^2*(a4_1 + a5_1 + g*mp*cos(a3_2)*sin(a3_2) + a3_4^2*l*mp*sin(a3_2)))/(mc + mp - mp*cos(a3_2)^2)^2 - (2*mp*cos(a3_2)^2*(a4_1 + a5_1 + g*mp*cos(a3_2)*sin(a3_2) + a3_4^2*l*mp*sin(a3_2)))/(mc + mp - mp*cos(a3_2)^2)^2 - (mp*(4*g*cos(a3_2)*sin(a3_2) + a3_4^2*l*sin(a3_2)))/(mc + mp - mp*cos(a3_2)^2) - (4*mp^2*cos(a3_2)*sin(a3_2)*(2*g*cos(a3_2)^2 - g + a3_4^2*l*cos(a3_2)))/(mc + mp - mp*cos(a3_2)^2)^2 + (8*mp^2*cos(a3_2)^2*sin(a3_2)^2*(a4_1 + a5_1 + g*mp*cos(a3_2)*sin(a3_2) + a3_4^2*l*mp*sin(a3_2)))/(mc + mp - mp*cos(a3_2)^2)^3;
  d2f(3,19) = (2*a3_4*l*mp*cos(a3_2)*(mc - mp*sin(a3_2)^2))/(mc + mp*sin(a3_2)^2)^2;
  d2f(3,20) = -(4*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(3,21) = -(4*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(3,31) = (2*a3_4*l*mp*cos(a3_2)*(mc - mp*sin(a3_2)^2))/(mc + mp - mp*cos(a3_2)^2)^2;
  d2f(3,33) = (2*l*mp*sin(a3_2))/(mc + mp*sin(a3_2)^2);
  d2f(3,38) = -(4*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(3,45) = -(4*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(4,17) = (a4_1*mc^2*cos(a3_2) + a5_1*mc^2*cos(a3_2) - (11*a4_1*mp^2*cos(a3_2))/8 - (11*a5_1*mp^2*cos(a3_2))/8 + g*mc^3*sin(a3_2) - (7*g*mp^3*sin(a3_2))/8 + (21*a4_1*mp^2*cos(3*a3_2))/16 + (a4_1*mp^2*cos(5*a3_2))/16 + (21*a5_1*mp^2*cos(3*a3_2))/16 + (a5_1*mp^2*cos(5*a3_2))/16 + (3*g*mp^3*sin(3*a3_2))/16 + (g*mp^3*sin(5*a3_2))/16 + (27*g*mc*mp^2*sin(3*a3_2))/16 + (3*g*mc^2*mp*sin(3*a3_2))/2 + (g*mc*mp^2*sin(5*a3_2))/16 + (a4_1*mc*mp*cos(a3_2))/2 + (a5_1*mc*mp*cos(a3_2))/2 - (a3_4^2*l*mp^3*sin(2*a3_2))/2 + (a3_4^2*l*mp^3*sin(4*a3_2))/4 + (3*a4_1*mc*mp*cos(3*a3_2))/2 + (3*a5_1*mc*mp*cos(3*a3_2))/2 + (5*g*mc*mp^2*sin(a3_2))/8 + (5*g*mc^2*mp*sin(a3_2))/2 + 2*a3_4^2*l*mc*mp^2*sin(2*a3_2) + 2*a3_4^2*l*mc^2*mp*sin(2*a3_2) + (a3_4^2*l*mc*mp^2*sin(4*a3_2))/2)/(l*(mc + mp - mp*cos(a3_2)^2)^3);
  d2f(4,19) = -(4*a3_4*mp*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(4,20) = (sin(a3_2)*(mc + 2*mp - mp*sin(a3_2)^2))/(l*(mc + mp*sin(a3_2)^2)^2);
  d2f(4,21) = (sin(a3_2)*(mc + 2*mp - mp*sin(a3_2)^2))/(l*(mc + mp*sin(a3_2)^2)^2);
  d2f(4,31) = -(a3_4*mp*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(mc + mp*sin(a3_2)^2)^2;
  d2f(4,33) = -(2*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2));
  d2f(4,38) = (sin(a3_2)*(mc + 2*mp - mp*sin(a3_2)^2))/(l*(mc + mp*sin(a3_2)^2)^2);
  d2f(4,45) = (sin(a3_2)*(mc + 2*mp - mp*sin(a3_2)^2))/(l*(mc + mp*sin(a3_2)^2)^2);
else
  d2f=[];
end  % if (order>=2)


 % NOTEST
