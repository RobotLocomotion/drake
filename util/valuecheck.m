function tf = valuecheck(val,desired_val,tol)

% VALUECHECK
%   Usage:  valuecheck(val,desired_val)

if (nargin<3) tol=1e-8; end

tf = true;

if isscalar(desired_val)
  desired_val = repmat(desired_val,size(val));
end

if ((length(size(val))~=length(size(desired_val))) || any(size(val)~=size(desired_val)))
  if (nargout>0)
    tf = false;
    warning(['Wrong size.  Expected ', mat2str(size(desired_val)),' but got ', mat2str(size(val))]);
    return;
  else
    error(['Wrong size.  Expected ', mat2str(size(desired_val)),' but got ', mat2str(size(val))]);
  end
end

if (any(abs(val(:)-desired_val(:))>tol))
  if (nargout>0)
    tf = false;
%    warning(['Values don''t match.  Expected ', mat2str(desired_val), ' but got ', mat2str(val)]);
  else
    desired_val-val
    error(['Values don''t match.  Expected ', mat2str(desired_val), ' but got ', mat2str(val)]);
  end
end


