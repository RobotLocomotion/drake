function [df] = dynamicsGradients(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'CompassGaitStancePlant');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[4  1]);
sizecheck(a4,[1  1]);

% Symbol table:
a=a1.a;
a3_1=a3(1);
a3_2=a3(2);
a3_3=a3(3);
a3_4=a3(4);
a4_1=a4(1);
b=a1.b;
g=a1.g;
l=a1.l;
m=a1.m;
mh=a1.mh;


% Compute Gradients:
df = sparse(4,6);
df(1,4) = 1;
df(2,5) = 1;
df(3,2) = (2*l^2*cos(a3_1 - a3_2)*sin(a3_1 - a3_2)*(a^2*a4_1*m + a4_1*l^2*m + a4_1*l^2*mh - a3_4^2*b*l^3*m^2*sin(a3_1 - a3_2) + a^2*b*g*m^2*sin(a3_1) + b*g*l^2*m^2*sin(a3_1) + (a3_3^2*b^2*l^2*m^2*sin(2*a3_1 - 2*a3_2))/2 - a4_1*b*l*m*cos(a3_1 - a3_2) - a3_4^2*b*l^3*m*mh*sin(a3_1 - a3_2) - a^2*a3_4^2*b*l*m^2*sin(a3_1 - a3_2) - b*g*l^2*m^2*sin(a3_2)*cos(a3_1 - a3_2) + b*g*l^2*m*mh*sin(a3_1) - a*b*g*l*m^2*sin(a3_2)*cos(a3_1 - a3_2) - b*g*l^2*m*mh*sin(a3_2)*cos(a3_1 - a3_2)))/(b^2*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2)^2) - (a4_1*l*sin(a3_1 - a3_2) - a3_4^2*l^3*m*cos(a3_1 - a3_2) - a3_4^2*l^3*mh*cos(a3_1 - a3_2) + a^2*g*m*cos(a3_1) + g*l^2*m*cos(a3_1) + g*l^2*mh*cos(a3_1) - a^2*a3_4^2*l*m*cos(a3_1 - a3_2) + g*l^2*m*sin(a3_2)*sin(a3_1 - a3_2) + g*l^2*mh*sin(a3_2)*sin(a3_1 - a3_2) + a3_3^2*b*l^2*m*cos(2*a3_1 - 2*a3_2) + a*g*l*m*sin(a3_2)*sin(a3_1 - a3_2))/(b*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2));
df(3,3) = (2*a4_1*l*sin(a3_1 - a3_2) - 2*a3_4^2*l^3*m*cos(a3_1 - a3_2) - 2*a3_4^2*l^3*mh*cos(a3_1 - a3_2) + 2*g*l^2*m*cos(a3_1 - 2*a3_2) + 2*g*l^2*mh*cos(a3_1 - 2*a3_2) - 2*a^2*a3_4^2*l*m*cos(a3_1 - a3_2) + 2*a*g*l*m*cos(a3_1 - 2*a3_2) + 2*a3_3^2*b*l^2*m*cos(2*a3_1 - 2*a3_2))/(b*(2*a^2*m + l^2*m + 2*l^2*mh - l^2*m*cos(2*a3_1 - 2*a3_2))) - (2*l^2*cos(a3_1 - a3_2)*sin(a3_1 - a3_2)*(a^2*a4_1*m + a4_1*l^2*m + a4_1*l^2*mh - a3_4^2*b*l^3*m^2*sin(a3_1 - a3_2) + a^2*b*g*m^2*sin(a3_1) + b*g*l^2*m^2*sin(a3_1) + (a3_3^2*b^2*l^2*m^2*sin(2*a3_1 - 2*a3_2))/2 - a4_1*b*l*m*cos(a3_1 - a3_2) - a3_4^2*b*l^3*m*mh*sin(a3_1 - a3_2) - a^2*a3_4^2*b*l*m^2*sin(a3_1 - a3_2) - b*g*l^2*m^2*sin(a3_2)*cos(a3_1 - a3_2) + b*g*l^2*m*mh*sin(a3_1) - a*b*g*l*m^2*sin(a3_2)*cos(a3_1 - a3_2) - b*g*l^2*m*mh*sin(a3_2)*cos(a3_1 - a3_2)))/(b^2*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2)^2);
df(3,4) = -(a3_3*l^2*m*sin(2*a3_1 - 2*a3_2))/(m*a^2 + m*l^2*sin(a3_1 - a3_2)^2 + mh*l^2);
df(3,5) = (2*a3_4*l*sin(a3_1 - a3_2)*(a^2*m + l^2*m + l^2*mh))/(b*(m*a^2 + m*l^2*sin(a3_1 - a3_2)^2 + mh*l^2));
df(3,6) = -(a^2*m + l^2*m + l^2*mh - b*l*m*cos(a3_1 - a3_2))/(b^2*m*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2));
df(4,2) = (- m*cos(a3_1 - a3_2)*a3_3^2*b^2*l + m*cos(2*a3_1 - 2*a3_2)*a3_4^2*b*l^2 - g*m*cos(2*a3_1 - a3_2)*b*l + a4_1*sin(a3_1 - a3_2)*l)/(b*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2)) - (l^2*m*cos(a3_1 - a3_2)*sin(a3_1 - a3_2)*(2*a4_1*b - 2*a4_1*l*cos(a3_1 - a3_2) + a3_4^2*b*l^2*m*sin(2*a3_1 - 2*a3_2) - b*g*l*m*sin(2*a3_1 - a3_2) - 2*a3_3^2*b^2*l*m*sin(a3_1 - a3_2) + 2*a*b*g*m*sin(a3_2) + b*g*l*m*sin(a3_2) + 2*b*g*l*mh*sin(a3_2)))/(b*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2)^2);
df(4,3) = (b*g*l*m*cos(2*a3_1 - a3_2) - 2*a4_1*l*sin(a3_1 - a3_2) + 2*a3_3^2*b^2*l*m*cos(a3_1 - a3_2) + 2*a*b*g*m*cos(a3_2) + b*g*l*m*cos(a3_2) + 2*b*g*l*mh*cos(a3_2) - 2*a3_4^2*b*l^2*m*cos(2*a3_1 - 2*a3_2))/(2*b*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2)) + (l^2*m*cos(a3_1 - a3_2)*sin(a3_1 - a3_2)*(2*a4_1*b - 2*a4_1*l*cos(a3_1 - a3_2) + a3_4^2*b*l^2*m*sin(2*a3_1 - 2*a3_2) - b*g*l*m*sin(2*a3_1 - a3_2) - 2*a3_3^2*b^2*l*m*sin(a3_1 - a3_2) + 2*a*b*g*m*sin(a3_2) + b*g*l*m*sin(a3_2) + 2*b*g*l*mh*sin(a3_2)))/(b*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2)^2);
df(4,4) = -(2*a3_3*b*l*m*sin(a3_1 - a3_2))/(m*a^2 + m*l^2*sin(a3_1 - a3_2)^2 + mh*l^2);
df(4,5) = (a3_4*l^2*m*sin(2*a3_1 - 2*a3_2))/(m*a^2 + m*l^2*sin(a3_1 - a3_2)^2 + mh*l^2);
df(4,6) = (b - l*cos(a3_1 - a3_2))/(b*(a^2*m + l^2*m + l^2*mh - l^2*m*cos(a3_1 - a3_2)^2));


 % NOTEST
