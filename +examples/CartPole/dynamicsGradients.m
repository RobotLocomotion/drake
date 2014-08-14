function [df, d2f, d3f] = dynamicsGradients(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'CartPolePlant');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[4  1]);
sizecheck(a4,[1  1]);

% Symbol table:
a3_2=a3(2);
a3_3=a3(3);
a3_4=a3(4);
a4_1=a4(1);
g=a1.g;
l=a1.l;
mc=a1.mc;
mp=a1.mp;


% Compute Gradients:
df = sparse(4,6);
df(1,4) = 1;
df(2,5) = 1;
df(3,3) = (l*mp*cos(a3_2)*a3_4^2 + g*mp*cos(2*a3_2))/(mc + mp/2 - (mp*cos(2*a3_2))/2) - (mp*sin(2*a3_2)*(l*mp*sin(a3_2)*a3_4^2 + a4_1 + (g*mp*sin(2*a3_2))/2))/(mc + mp/2 - (mp*cos(2*a3_2))/2)^2;
df(3,5) = (2*a3_4*l*mp*sin(a3_2))/(mp*sin(a3_2)^2 + mc);
df(3,6) = 1/(mp*sin(a3_2)^2 + mc);
df(4,3) = (mp*sin(2*a3_2)*((l*mp*sin(2*a3_2)*a3_4^2)/2 + a4_1*cos(a3_2) + g*mc*sin(a3_2) + g*mp*sin(a3_2)))/(l*(mc + mp/2 - (mp*cos(2*a3_2))/2)^2) - (l*mp*cos(2*a3_2)*a3_4^2 - a4_1*sin(a3_2) + g*mc*cos(a3_2) + g*mp*cos(a3_2))/(l*(mc + mp/2 - (mp*cos(2*a3_2))/2));
df(4,5) = -(2*a3_4*mp*cos(a3_2)*sin(a3_2))/(mp*sin(a3_2)^2 + mc);
df(4,6) = -cos(a3_2)/(l*(- mp*cos(a3_2)^2 + mc + mp));

% d2f
if (order>=2)
  d2f = sparse(4,36);
  d2f(3,15) = (16*mp^2*sin(2*a3_2)^2*(l*mp*sin(a3_2)*a3_4^2 + a4_1 + (g*mp*sin(2*a3_2))/2))/(2*mc + mp - mp*cos(2*a3_2))^3 - (l*mp*sin(a3_2)*a3_4^2 + 4*g*mp*cos(a3_2)*sin(a3_2))/(- mp*cos(a3_2)^2 + mc + mp) - (8*mp^2*sin(2*a3_2)*(l*a3_4^2*cos(a3_2) + 2*g*cos(a3_2)^2 - g))/(2*mc + mp - mp*cos(2*a3_2))^2 - (8*mp*cos(2*a3_2)*(l*mp*sin(a3_2)*a3_4^2 + a4_1 + (g*mp*sin(2*a3_2))/2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(3,17) = (2*a3_4*l*mp*cos(a3_2)*(mc - mp*sin(a3_2)^2))/(mp*sin(a3_2)^2 + mc)^2;
  d2f(3,18) = -(2*mp*cos(a3_2)*sin(a3_2))/(mp*sin(a3_2)^2 + mc)^2;
  d2f(3,27) = (2*a3_4*l*mp*cos(a3_2)*(mp*cos(a3_2)^2 + mc - mp))/(- mp*cos(a3_2)^2 + mc + mp)^2;
  d2f(3,29) = (2*l*mp*sin(a3_2))/(mp*sin(a3_2)^2 + mc);
  d2f(3,33) = -(4*mp*sin(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(4,15) = (4*l*mp*cos(a3_2)*sin(a3_2)*a3_4^2 + a4_1*cos(a3_2) + g*mc*sin(a3_2) + g*mp*sin(a3_2))/(l*(- mp*cos(a3_2)^2 + mc + mp)) + (8*mp*cos(2*a3_2)*(l*mp*cos(a3_2)*sin(a3_2)*a3_4^2 + a4_1*cos(a3_2) + g*mc*sin(a3_2) + g*mp*sin(a3_2)))/(l*(2*mc + mp - mp*cos(2*a3_2))^2) + (8*mp*sin(2*a3_2)*(l*mp*cos(2*a3_2)*a3_4^2 - a4_1*sin(a3_2) + g*mc*cos(a3_2) + g*mp*cos(a3_2)))/(l*(2*mc + mp - mp*cos(2*a3_2))^2) - (16*mp^2*sin(2*a3_2)^2*(l*mp*cos(a3_2)*sin(a3_2)*a3_4^2 + a4_1*cos(a3_2) + g*mc*sin(a3_2) + g*mp*sin(a3_2)))/(l*(2*mc + mp - mp*cos(2*a3_2))^3);
  d2f(4,17) = -(a3_4*mp*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(mp*sin(a3_2)^2 + mc)^2;
  d2f(4,18) = (sin(a3_2)*(- mp*sin(a3_2)^2 + mc + 2*mp))/(l*(mp*sin(a3_2)^2 + mc)^2);
  d2f(4,27) = -(4*a3_4*mp*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d2f(4,29) = -(2*mp*cos(a3_2)*sin(a3_2))/(mp*sin(a3_2)^2 + mc);
  d2f(4,33) = (sin(a3_2)*(- mp*sin(a3_2)^2 + mc + 2*mp))/(l*(mp*sin(a3_2)^2 + mc)^2);
else
  d2f=[];
end  % if (order>=2)

% d3f
if (order>=3)
  d3f = sparse(4,216);
  d3f(3,87) = (8*mp^2*sin(2*a3_2)*(l*sin(a3_2)*a3_4^2 + 2*g*sin(2*a3_2)))/(2*mc + mp - mp*cos(2*a3_2))^2 - (mp*(l*a3_4^2*cos(a3_2) + 8*g*cos(a3_2)^2 - 4*g))/(- mp*cos(a3_2)^2 + mc + mp) + (48*mp^3*sin(2*a3_2)^2*(l*a3_4^2*cos(a3_2) + 2*g*cos(a3_2)^2 - g))/(2*mc + mp - mp*cos(2*a3_2))^3 - (24*mp^2*cos(2*a3_2)*(l*a3_4^2*cos(a3_2) + 2*g*cos(a3_2)^2 - g))/(2*mc + mp - mp*cos(2*a3_2))^2 - (96*mp^3*sin(2*a3_2)^3*(l*mp*sin(a3_2)*a3_4^2 + a4_1 + (g*mp*sin(2*a3_2))/2))/(2*mc + mp - mp*cos(2*a3_2))^4 + (16*mp*sin(2*a3_2)*(l*mp*sin(a3_2)*a3_4^2 + a4_1 + (g*mp*sin(2*a3_2))/2))/(2*mc + mp - mp*cos(2*a3_2))^2 + (2*mp^2*cos(a3_2)*sin(a3_2)*(l*sin(a3_2)*a3_4^2 + 2*g*sin(2*a3_2)))/(- mp*cos(a3_2)^2 + mc + mp)^2 + (96*mp^2*cos(2*a3_2)*sin(2*a3_2)*(l*mp*sin(a3_2)*a3_4^2 + a4_1 + (g*mp*sin(2*a3_2))/2))/(2*mc + mp - mp*cos(2*a3_2))^3;
  d3f(3,89) = -(2*a3_4*l*mp*sin(a3_2)*(mc^2 - 6*mc*mp*sin(a3_2)^2 + 6*mc*mp + mp^2*sin(a3_2)^4 - 2*mp^2*sin(a3_2)^2))/(mp*sin(a3_2)^2 + mc)^3;
  d3f(3,90) = -(2*mp*(mc - 2*mc*sin(a3_2)^2 - 3*mp*sin(a3_2)^2 + 2*mp*sin(a3_2)^4))/(mp*sin(a3_2)^2 + mc)^3;
  d3f(3,99) = -(2*a3_4*l*mp*sin(a3_2)*(mc^2 - 6*mc*mp*sin(a3_2)^2 + 6*mc*mp + mp^2*sin(a3_2)^4 - 2*mp^2*sin(a3_2)^2))/(mp*sin(a3_2)^2 + mc)^3;
  d3f(3,101) = (2*l*mp*cos(a3_2)*(mc - mp*sin(a3_2)^2))/(mp*sin(a3_2)^2 + mc)^2;
  d3f(3,105) = (16*mp^2*sin(2*a3_2)^2)/(2*mc + mp - mp*cos(2*a3_2))^3 - (8*mp*cos(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d3f(3,159) = -(2*a3_4*l*mp*sin(a3_2)*(mc^2 - 6*mc*mp*sin(a3_2)^2 + 6*mc*mp + mp^2*sin(a3_2)^4 - 2*mp^2*sin(a3_2)^2))/(mp*sin(a3_2)^2 + mc)^3;
  d3f(3,161) = (2*l*mp*cos(a3_2)*(mc - mp*sin(a3_2)^2))/(mp*sin(a3_2)^2 + mc)^2;
  d3f(3,171) = (2*l*mp*cos(a3_2)*(mp*cos(a3_2)^2 + mc - mp))/(- mp*cos(a3_2)^2 + mc + mp)^2;
  d3f(3,195) = (16*mp^2*sin(2*a3_2)^2)/(2*mc + mp - mp*cos(2*a3_2))^3 - (8*mp*cos(2*a3_2))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d3f(4,87) = (8*l*a3_4^2*mc^3*mp*cos(a3_2)^2 - 4*l*a3_4^2*mc^3*mp + 32*l*a3_4^2*mc^2*mp^2*cos(a3_2)^4 - 20*l*a3_4^2*mc^2*mp^2*cos(a3_2)^2 - 6*l*a3_4^2*mc^2*mp^2 + 8*l*a3_4^2*mc*mp^3*cos(a3_2)^6 + 20*l*a3_4^2*mc*mp^3*cos(a3_2)^4 - 28*l*a3_4^2*mc*mp^3*cos(a3_2)^2 + 4*l*a3_4^2*mp^4*cos(a3_2)^6 - 6*l*a3_4^2*mp^4*cos(a3_2)^4 + 2*l*a3_4^2*mp^4 + g*mc^4*cos(a3_2) + 23*g*mc^3*mp*cos(a3_2)^3 - 16*g*mc^3*mp*cos(a3_2) - a4_1*mc^3*sin(a3_2) + 23*g*mc^2*mp^2*cos(a3_2)^5 + 13*g*mc^2*mp^2*cos(a3_2)^3 - 30*g*mc^2*mp^2*cos(a3_2) + 23*a4_1*mc^2*mp*sin(a3_2)^3 - 20*a4_1*mc^2*mp*sin(a3_2) + g*mc*mp^3*cos(a3_2)^7 + 26*g*mc*mp^3*cos(a3_2)^5 - 19*g*mc*mp^3*cos(a3_2)^3 - 8*g*mc*mp^3*cos(a3_2) - 23*a4_1*mc*mp^2*cos(a3_2)^4*sin(a3_2) + 10*a4_1*mc*mp^2*sin(a3_2)^3 - a4_1*mc*mp^2*sin(a3_2) + g*mp^4*cos(a3_2)^7 + 3*g*mp^4*cos(a3_2)^5 - 9*g*mp^4*cos(a3_2)^3 + 5*g*mp^4*cos(a3_2) - a4_1*mp^3*cos(a3_2)^6*sin(a3_2) - 17*a4_1*mp^3*cos(a3_2)^4*sin(a3_2) - 13*a4_1*mp^3*sin(a3_2)^3 + 18*a4_1*mp^3*sin(a3_2))/(l*(- mp*cos(a3_2)^2 + mc + mp)^4);
  d3f(4,89) = (2*a3_4*mp*sin(2*a3_2)*(2*mc + mp))/(mp*sin(a3_2)^2 + mc)^2 + (4*a3_4*mp^2*cos(a3_2)*sin(a3_2)*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(mp*sin(a3_2)^2 + mc)^3;
  d3f(4,90) = (cos(a3_2)*(mc^2 - 6*mc*mp*sin(a3_2)^2 + 2*mc*mp + mp^2*sin(a3_2)^4 - 6*mp^2*sin(a3_2)^2))/(l*(mp*sin(a3_2)^2 + mc)^3);
  d3f(4,99) = (16*a3_4*mp^2*sin(2*a3_2)*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(2*mc + mp - mp*cos(2*a3_2))^3 + (8*a3_4*mp*sin(2*a3_2)*(2*mc + mp))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d3f(4,101) = -(mp^2*cos(2*a3_2) - mp^2 + 2*mc*mp*cos(2*a3_2))/(mp*sin(a3_2)^2 + mc)^2;
  d3f(4,105) = (cos(a3_2)*(mc^2 - 6*mc*mp*sin(a3_2)^2 + 2*mc*mp + mp^2*sin(a3_2)^4 - 6*mp^2*sin(a3_2)^2))/(l*(mp*sin(a3_2)^2 + mc)^3);
  d3f(4,159) = (a3_4*mp*(8*mc^2*sin(2*a3_2) - 2*mp^2*sin(2*a3_2) + mp^2*sin(4*a3_2) + 8*mc*mp*sin(2*a3_2) + 2*mc*mp*sin(4*a3_2)))/(2*(- mp*cos(a3_2)^2 + mc + mp)^3);
  d3f(4,161) = -(mp*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(mp*sin(a3_2)^2 + mc)^2;
  d3f(4,171) = -(4*mp*(2*mc*cos(2*a3_2) - mp + mp*cos(2*a3_2)))/(2*mc + mp - mp*cos(2*a3_2))^2;
  d3f(4,195) = cos(a3_2)/(l*(- mp*cos(a3_2)^2 + mc + mp)) - (16*mp^2*sin(2*a3_2)^2*cos(a3_2))/(l*(2*mc + mp - mp*cos(2*a3_2))^3) + (8*mp*cos(2*a3_2)*cos(a3_2))/(l*(2*mc + mp - mp*cos(2*a3_2))^2) - (8*mp*sin(2*a3_2)*sin(a3_2))/(l*(2*mc + mp - mp*cos(2*a3_2))^2);
else
  d3f=[];
end  % if (order>=3)


 % NOTEST
