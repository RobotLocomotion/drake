function tf = inFunnel(V,varargin)
% For a "funnel" described by the one-sublevel set of an msspoly V, use
%   inFunnel(V,x)
% which returns a vector tf which is true or false for each column of V where
% true indicates that V(x)<=1
%
% For a "funnel" described by the one-sublevel set of a
% polynomialtrajectory V, use
%   inFunnel(Vtraj,t,x)
% which returns a vector tf which is true of false for each pair
% (t(:,i),x(:,i)), where t and x must have the same number of columns.
%
% @param V either an msspoly, or a PolynomialTrajectory, whose one-sublevel
% set describes the funnel.
% @param t (when V is a PolynomialTrajectory) row vector describing the times to evaluate 
% @param x states to evaluate, one per column.
%

if (isa(V,'msspoly'))  % then TI funnel
  if (length(varargin)<1) error('usage: inFunnel(V,x)'); end
  x = varargin{1};
  p_x=decomp(V);
  tf = doubleSafe(msubs(V,p_x,x))<=1;
elseif (isa(V,'PolynomialTrajectory')) % then TV funnel
  if (length(varargin)<2) error('usage: inFunnel(Vtraj,t,x)'); end
  t = varargin{1};
  x = varargin{2};
  if (size(t,2)~=size(x,2)) error('t and x must have the same number of columns'); end
  if (size(t,1)~=1) error('t must be a row vector'); end
  for i=1:length(t)
    tf(1,i) = doubleSafe(polyeval(V,t(i),x(:,i)))<=1;
  end
else
  error('V must be an msspoly or PolynomialTrajectory');
end

end  


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end
