function [df, d2f, d3f] = dynamicsGradients(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'PendulumPlant');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[2  1]);
sizecheck(a4,[1  1]);

% Symbol table:
I=a1.I;
a3_1=a3(1);
a3_2=a3(2);
a4_1=a4(1);
b=a1.b;
g=a1.g;
lc=a1.lc;
m=a1.m;


% Compute Gradients:
df = sparse(2,4);
df(1,3) = 1;
df(2,2) = -(g*lc*m*cos(a3_1))/I;
df(2,3) = -b/I;
df(2,4) = 1/I;

% d2f
if (order>=2)
  d2f = sparse(2,16);
  d2f(2,6) = (g*lc*m*sin(a3_1))/I;
else
  d2f=[];
end  % if (order>=2)

% d3f
if (order>=3)
  d3f = sparse(2,64);
  d3f(2,22) = (g*lc*m*cos(a3_1))/I;
else
  d3f=[];
end  % if (order>=3)


 % NOTEST
