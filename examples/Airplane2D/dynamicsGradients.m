function [df, d2f, d3f] = dynamicsGradients(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'PlanePlant');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[4  1]);
sizecheck(a4,[1  1]);

% Symbol table:
a3_3=a3(3);
a3_4=a3(4);
a4_1=a4(1);
v = a1.v;

% Compute Gradients:
df = sparse(4,6);
df(1,4) = -v*cos(a3_3);
df(2,4) = -v*sin(a3_3);
df(3,5) = 1;
df(4,6) = 1;

% d2f
if (order>=2)
  d2f = sparse(4,36);
  d2f(1,22) = v*sin(a3_3);
  d2f(2,22) = -v*cos(a3_3);
else
  d2f=[];
end  % if (order>=2)

% d3f
if (order>=3)
  d3f = sparse(4,216);
  d3f(1,130) = v*cos(a3_3);
  d3f(2,130) = v*sin(a3_3);
else
  d3f=[];
end  % if (order>=3)


 % NOTEST
