function valuecheck(val,desired_val,tol)

% VALUECHECK
%   Usage:  valuecheck(val,desired_val)

if (nargin<3) tol=1e-8; end

if ((length(size(val))~=length(size(desired_val))) || any(size(val)~=size(desired_val)))
  error(['Wrong size.  Expected ', mat2str(size(desired_val)),' but got ', mat2str(size(val))]);
end

if (any(abs(val(:)-desired_val(:))>tol))
  error(['Values don''t match.  Expected ', mat2str(desired_val), ' but got ', mat2str(val)]);
end


