function [df, d2f, d3f] = dynamicsGradients(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'DubinsPlant');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[3  1]);
sizecheck(a4,[1  1]);

% Symbol table:
a3_3=a3(3);
a4_1=a4(1);
v=a1.v;


% Compute Gradients:
df = sparse(3,5);
df(1,4) = -v*sin(a3_3);
df(2,4) = v*cos(a3_3);
df(3,5) = 1;

% d2f
if (order>=2)
  d2f = sparse(3,25);
  d2f(1,19) = -v*cos(a3_3);
  d2f(2,19) = -v*sin(a3_3);
else
  d2f=[];
end  % if (order>=2)

% d3f
if (order>=3)
  d3f = sparse(3,125);
  d3f(1,94) = v*sin(a3_3);
  d3f(2,94) = -v*cos(a3_3);
else
  d3f=[];
end  % if (order>=3)


 % NOTEST
